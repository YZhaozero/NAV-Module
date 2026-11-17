"""Read every message of every rosbag2 split; never publish into a live ROS graph."""
from collections import Counter
from pathlib import Path
import argparse
import json
import hashlib
import time
import yaml
from .capture_quality import StreamStats, FramePairs, header_stamp, atomic_json, STATIC_TYPES, capture_quality_warnings, OPTIONAL_RTK_TOPICS, STAMPLESS_DIAGNOSTIC_TOPICS, optional_capture_topics
from .bag_cache import release_file_cache
from .image_archive import archive_routes, FrameNumberWindow


def _messages(root, info, reader_factory):
    if reader_factory:
        reader = reader_factory(root, info)
        while reader.has_next():
            yield reader.read_next()
        return
    import rosbag2_py
    compressed = str(info.get("compression_mode", "")).lower() in ("file", "message")
    # Read a closed SQLite split at a time so audit does not charge the entire
    # bag to the app's cgroup again immediately after the writer released it.
    split_reads = not compressed and info["storage_identifier"] in ("sqlite3", "mcap")
    uris = [root / name for name in info["relative_file_paths"]] if split_reads else [root]
    cls = rosbag2_py.SequentialCompressionReader if compressed else rosbag2_py.SequentialReader
    for uri in uris:
        reader = cls()
        try:
            reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=info["storage_identifier"]),
                        rosbag2_py.ConverterOptions("", ""))
            while reader.has_next():
                yield reader.read_next()
        finally:
            del reader
            if split_reads:
                release_file_cache(uri)


def audit_bag(directory, selected=(), max_gap=2., reader_factory=None, *, progress=None, cancelled=None):
    root = Path(directory).resolve()
    info = yaml.safe_load((root / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    errors, warnings = [], []
    paths = info.get("relative_file_paths", [])
    if not paths:
        errors.append("元数据未列出分片")
    for name in paths:
        path = (root / name).resolve()
        if root not in path.parents or not path.is_file() or path.stat().st_size == 0:
            errors.append(f"分片缺失或无效：{name}")
    expected = {r["topic_metadata"]["name"]: int(r["message_count"])
                for r in info.get("topics_with_message_count", [])}
    types = {r["topic_metadata"]["name"]: r["topic_metadata"]["type"]
             for r in info.get("topics_with_message_count", [])}
    manifest = root/"nav_manifest.json"
    document = json.loads(manifest.read_text(encoding="utf-8")) if manifest.is_file() else {}
    protocol = document.get('capture_protocol', {})
    routes = archive_routes(root)
    archived = {r['stored_topic']: r for r in routes}
    declared_window=document.get('image_archive',{}).get('recording_window')
    archive_window=None
    if declared_window is not None:
        values=[declared_window.get(key) for key in ('first_stamp_ns','last_stamp_ns','accepted','published',
            'first_published_stamp_ns','last_published_stamp_ns')]
        if (declared_window.get('schema')!=1 or any(type(v) is not int or v<=0 for v in values)
                or values[0]>values[1] or values[4]>values[5]):
            errors.append('彩色归档接收窗口无效，无法确认完整录制范围')
        else:
            archive_window=(values[0],values[1])
            if values[2]!=values[3] or archive_window!=(values[4],values[5]):
                errors.append('彩色归档接收与发布数量或头戳范围不一致')
    metadata_routes = {r['source_topic'].rsplit('/', 1)[0]+'/metadata': r['stored_topic'] for r in routes}
    windows = {name: FrameNumberWindow() for name in archived}
    image_checks = {name: dict(decoded=0, dimensions=None) for name in archived}
    optional = optional_capture_topics(protocol)
    for name in selected:
        if expected.get(name, 0) == 0:
            (warnings if name in optional else errors).append(f"所选话题未录到数据：{name}")
    stats = {name: StreamStats(timestamp_required=STAMPLESS_DIAGNOSTIC_TOPICS.get(name) != typ)
             for name, typ in types.items()}
    pairs = {name: name.rsplit("/",1)[0]+"/camera_info" for name,typ in types.items()
             if typ == "sensor_msgs/msg/Image" and name.rsplit("/",1)[0]+"/camera_info" in types}
    if manifest.is_file():
        roles = protocol.get("roles",{})
        if roles.get("rgb") in types and roles.get("camera_info") in types:
            pairs[roles["rgb"]] = roles["camera_info"]
    joins = {name:FramePairs(window=archive_window if name in archived else None) for name in pairs}
    paired = {}
    frame_numbers = {}
    source_retention = {}
    processed = 0
    total = int(info.get('message_count', 0))
    last_progress = 0.
    def checkpoint(force=False):
        nonlocal last_progress
        if cancelled is not None and cancelled():
            raise InterruptedError('录包校验已暂停，原始数据保留')
        now = time.monotonic()
        if progress is not None and (force or now-last_progress >= .25):
            progress(processed, total)
            last_progress = now
    checkpoint(True)
    if not errors:
        try:
            for name, data, timestamp in _messages(root, info, reader_factory):
                checkpoint()
                processed += 1
                if name not in stats:
                    raise ValueError(f"元数据未声明话题：{name}")
                stats[name].add(data, types[name], timestamp / 1e9)
                if name in archived:
                    import cv2
                    from rclpy.serialization import deserialize_message
                    from sensor_msgs.msg import CompressedImage
                    from .image_archive import decode_image
                    cv2.setNumThreads(1)
                    message = decode_image(deserialize_message(data, CompressedImage))
                    dimensions = [message.width, message.height]
                    check = image_checks[name]
                    if check['dimensions'] not in (None, dimensions):
                        raise ValueError('录包彩色图像尺寸变化')
                    calibration = document.get('calibration_start', {}).get(pairs.get(name), {})
                    if calibration and dimensions != [calibration.get('width'), calibration.get('height')]:
                        raise ValueError('归档图像尺寸与相机标定不一致')
                    check['dimensions'] = dimensions
                    check['decoded'] += 1
                    windows[name].image(header_stamp(data, types[name]))
                if types[name] == "realsense2_camera_msgs/msg/Metadata":
                    from rclpy.serialization import deserialize_message
                    from realsense2_camera_msgs.msg import Metadata
                    metadata = json.loads(deserialize_message(data, Metadata).json_data)
                    number = metadata.get("frame_number")
                    if not isinstance(number, int):
                        errors.append(f"相机元数据缺少帧编号：{name}")
                    elif name in metadata_routes:
                        windows[metadata_routes[name]].metadata(header_stamp(data, types[name]), number)
                    else:
                        counter = frame_numbers.setdefault(name, {"first":number,"last":number-1,"count":0,"non_increasing":0})
                        counter["non_increasing"] += int(number <= counter["last"])
                        counter["last"] = number
                        counter["count"] += 1

                for image,info_topic in pairs.items():
                    if name in (image,info_topic):
                        joins[image].add(header_stamp(data,types[name]), name == image)
            paired = {name:join.document() for name,join in joins.items()}
            for name, row in paired.items():
                if name in archived and archive_window is not None and (row['missing_images'] or row['missing_camera_info']):
                    errors.append(f"彩色归档接收窗口内图像与相机信息配对缺失：{name}，图像 {row['missing_images']} / 相机信息 {row['missing_camera_info']}")
                elif row["image_retention"] is None or row["image_retention"] < .98:
                    (warnings if name in optional else errors).append(f"图像与相机信息配对不足：{name}，缺少 {row['missing_images']} 帧图像")
            if archive_window is not None:
                for name in archived:
                    row=stats.get(name)
                    if (row is None or row.count!=declared_window['published'] or
                            (row.first_stamp,row.last_stamp)!=(declared_window['first_published_stamp_ns'],declared_window['last_published_stamp_ns'])):
                        errors.append(f'已发布彩色归档未完整写入录包：{name}')
            for name, row in stats.items():
                if row.count != expected[name]:
                    errors.append(f"消息数不一致：{name}，实际 {row.count} / 元数据 {expected[name]}")
                if types[name] not in STATIC_TYPES:
                    issues = warnings if OPTIONAL_RTK_TOPICS.get(name) == types[name] else errors
                    if row.invalid_stamps or row.non_increasing:
                        issues.append(f"采集时间戳无效或重复/倒退：{name} ({row.invalid_stamps}/{row.non_increasing})")
                    if row.max_gap_s > max_gap:
                        issues.append(f"采集时间戳间隔过大：{name} ({row.max_gap_s:.3f}s)")
            for name, image in metadata_routes.items():
                if windows[image].counter:
                    frame_numbers[name] = windows[image].counter
            for name, counter in frame_numbers.items():
                image = name.rsplit("/",1)[0]+("/image_rect_raw" if "/depth/" in name else "/image_raw")
                image = metadata_routes.get(name, image)
                if image not in stats:
                    continue
                expected_frames = counter["last"]-counter["first"]+1
                # Header-pair checks detect individual missing images; the hardware frame
                # counter also detects frames missing from BOTH image and CameraInfo.
                captured = stats[image].count
                retained = min(1., captured/expected_frames) if expected_frames > 0 else 0.
                source_retention[image] = {**counter,"counter_expected_frames":expected_frames,
                    "captured_images":captured,"retention":retained,
                    "reference":("sensor frame-number span within archived image headers" if image in archived
                                 else "sensor frame-number span between first and last recorded Metadata")}
                if counter["non_increasing"] or retained < .98:
                    errors.append(f"相机源帧保留不足或帧编号重置：{image} ({retained:.2%})")
            if sum(r.count for r in stats.values()) != int(info.get("message_count", 0)):
                errors.append("总消息数与元数据不一致")
        except InterruptedError:
            raise
        except (OSError, RuntimeError, ValueError, TypeError, KeyError) as exc:
            errors.append(f"分片完整读回失败：{exc}")
    warnings.append("消息数一致仅证明文件完整；缺帧应结合发布端计数与逐话题时间戳评估")
    result = {"schema_version": 1, "complete": not errors, "errors": errors,
              "warnings": warnings, "split_count": len(paths), "frame_pairs": paired,
              "image_archive": image_checks, "source_frames": source_retention,
              "topics": {name: row.document() for name, row in stats.items()}}
    result["quality_warnings"] = capture_quality_warnings(result, protocol)
    checkpoint(True)
    atomic_json(root / "nav_audit.json", result)
    return result


def _source_identity(root):
    """Cheap immutable-close identity; the audit itself reads every payload."""
    metadata = (root/'metadata.yaml').read_bytes()
    info = yaml.safe_load(metadata)['rosbag2_bagfile_information']
    parts = []
    for name in info.get('relative_file_paths', []):
        source = (root/name).resolve(strict=True)
        if root not in source.parents or not source.is_file():
            raise ValueError('录包分片路径无效')
        st = source.stat()
        parts.append((name, st.st_size, st.st_mtime_ns))
    if not parts:
        raise ValueError('录包没有数据分片')
    return hashlib.sha256(metadata).hexdigest(), parts


def _audit_keyframes(root, document, *, progress=None, cancelled=None, reader_factory=None):
    """Sparse samples have deliberate gaps, unlike a raw SLAM recording."""
    info = yaml.safe_load((root/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    expected = {r['topic_metadata']['name']: int(r['message_count'])
                for r in info.get('topics_with_message_count', [])}
    stored = document.get('keyframes', {}).get('expected_topic_counts', expected)
    actual, done, total, last = Counter(), 0, int(info.get('message_count', 0)), 0.
    for topic, _data, _stamp in _messages(root, info, reader_factory):
        if cancelled is not None and cancelled():
            raise InterruptedError('关键帧数据校验已暂停，原始数据保留')
        actual[topic] += 1
        done += 1
        now = time.monotonic()
        if progress is not None and now-last >= .25:
            progress(done, total); last = now
    errors = []
    if dict(actual) != {k:v for k,v in expected.items() if v} or dict(actual) != {k:v for k,v in stored.items() if v} or done != total:
        errors.append('关键帧写入数量校验不一致')
    if cancelled is not None and cancelled():
        raise InterruptedError('关键帧数据校验已暂停，原始数据保留')
    if progress is not None: progress(done, total)
    result = dict(complete=not errors, profile='keyframes', errors=errors,
        warnings=document.get('observation_warnings', []), intentional_sampling=True,
        topic_counts=dict(actual), selected_keyframes=document.get('keyframes', {}).get('statistics', {}).get('saved', 0))
    atomic_json(root/'nav_audit.json', result)
    return result


def finalize_recording(directory, progress=None, cancelled=None, *, reader_factory=None, stage_progress=None):
    """Finalize a durably closed recording offline, without ROS subscriptions.

    Cancellation leaves storage/audit_pending intact. The task owner retries the
    stage later. `complete` is only published after actual readback succeeds.
    """
    root = Path(directory).expanduser().resolve(strict=True)
    path = root/'nav_manifest.json'
    def report(stage):
        def update(done, total):
            if stage_progress is not None:stage_progress(stage,done,total)
            elif progress is not None:progress(done,total)
        return update
    # Protect the fixed-name atomic manifest writer from a second executor.
    lock = (root/'.nav-finalize.lock').open('a+b')
    try:
        if __import__('os').name == 'posix':
            import fcntl
            try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError: raise ValueError('此录包正在由另一项任务校验') from None
        else:
            import msvcrt
            if not lock.seek(0, 2): lock.write(b'0'); lock.flush()
            lock.seek(0)
            try: msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
            except OSError: raise ValueError('此录包正在由另一项任务校验') from None
        document = json.loads(path.read_text(encoding='utf-8'))
        if (document.get('state') == 'complete' and document.get('audit', {}).get('complete')
                and document.get('diagnostics_state') not in ('pending', 'incomplete')):
            return document
        if document.get('state') not in ('stored', 'incomplete', 'complete') or not document.get('storage_flush_confirmed'):
            raise ValueError('录包尚未确认完成落盘，不能开始离线校验')
        original = path.read_bytes()
        before = _source_identity(root)
        def fingerprint(identity):
            fields={key:document.get(key) for key in ('recording_profile','selected_topics','capture_protocol','keyframes',
                'storage_errors','image_archive','calibration_start','parameters_start')}
            return hashlib.sha256(json.dumps([identity,fields],sort_keys=True).encode()).hexdigest()
        audit_path=root/'nav_audit.json'
        try:cached=json.loads(audit_path.read_text(encoding='utf-8'))
        except (OSError,ValueError):cached={}
        if cached.get('complete') and cached.get('_source_fingerprint')==fingerprint(before):
            result=cached
            total=sum(row.get('count',0) for row in result.get('topics',{}).values())
            if result.get('profile')=='keyframes':total=sum(result.get('topic_counts',{}).values())
            report('audit')(total,total)
        elif document.get('recording_profile') == 'keyframes':
            result = _audit_keyframes(root, document, progress=report('audit'), cancelled=cancelled,
                                      reader_factory=reader_factory)
        else:
            result = audit_bag(root, document.get('selected_topics', []),
                document.get('capture_protocol', {}).get('max_silence_s', 2.), reader_factory,
                progress=report('audit'), cancelled=cancelled)
        if before != _source_identity(root) or path.read_bytes() != original:
            raise ValueError('校验期间录包或保存记录发生变化，请重新检查')
        result['_source_fingerprint']=fingerprint(before)
        atomic_json(audit_path,result)
        if cancelled is not None and cancelled():
            raise InterruptedError('录包处理已暂停，原始数据保留')
        errors = list(dict.fromkeys([*document.get('storage_errors', document.get('errors', [])), *result['errors']]))
        document['audit'] = result
        document['audit_state'] = 'complete' if result['complete'] else 'incomplete'
        snapshot = document.get('diagnostic_snapshot')
        if snapshot is not None:
            from .capture_evidence import collect_evidence
            key=hashlib.sha256(json.dumps(snapshot,sort_keys=True).encode()).hexdigest()
            diagnostic_path=root/'nav_diagnostics.json'
            try:diagnostics=json.loads(diagnostic_path.read_text(encoding='utf-8'))
            except (OSError,ValueError):diagnostics={}
            reusable=diagnostics.get('_snapshot_fingerprint')==key and not diagnostics.get('errors')
            for row in diagnostics.get('files',[]):
                target=root/'diagnostics'/row['path']
                try:
                    st=target.stat()
                    reusable=reusable and target.resolve().is_relative_to((root/'diagnostics').resolve()) and [st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns]==row.get('_archive_identity')
                except OSError:reusable=False
            if reusable:
                report('evidence')(len(diagnostics.get('files',[])),len(snapshot.get('files',[])))
            else:
                diagnostics=collect_evidence(root,document.get('started_unix',0.),snapshot=snapshot,
                    cancelled=cancelled,progress=report('evidence'))
                for row in diagnostics['files']:
                    st=(root/'diagnostics'/row['path']).stat()
                    row['_archive_identity']=[st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns]
                diagnostics['_snapshot_fingerprint']=key
                atomic_json(diagnostic_path,diagnostics)
            document['diagnostics']=diagnostics
            document['diagnostics_complete'] = not document['diagnostics']['errors']
            document['diagnostics_state'] = 'complete' if document['diagnostics_complete'] else 'incomplete'
        # Compression is optional: a failed compression does not invalidate an
        # already audited original, and the originals remain usable on disk.
        if not errors and document.get('buffer_budget', {}).get('post_record_compression') == 'zstd':
            from .bag_compression import compress_closed_bag
            try:
                info = yaml.safe_load((root/'metadata.yaml').read_text())['rosbag2_bagfile_information']
                if str(info.get('compression_mode', '')).lower() in ('', 'none'):
                    document['final_compression'] = compress_closed_bag(root, progress=report('compress'), cancelled=cancelled)
                else:
                    document.setdefault('final_compression', dict(mode='file', format='zstd', recovered_commit=True))
            except InterruptedError:
                raise
            except (OSError, ValueError) as exc:
                document['compression_warning'] = str(exc)
            # Verified compression changes storage identity but not sensor data.
            # Reuse its successful readback checkpoint after a later interruption.
            result['_source_fingerprint']=fingerprint(_source_identity(root))
            atomic_json(audit_path,result)
        if cancelled is not None and cancelled():
            raise InterruptedError('录包处理已暂停，原始数据保留')
        document.update(state='incomplete' if errors else 'complete', errors=errors,
                        audited_unix=time.time())
        atomic_json(path, document)
        if snapshot is not None and document.get('diagnostics_complete'):
            # Only our own same-directory pins, after the report is durable.
            pins=(root/'.diagnostic-snapshot').resolve()
            for row in snapshot.get('files', []):
                if row.get('pin'):
                    pin=Path(row['pin'])
                    if pin.parent.resolve()==pins:pin.unlink(missing_ok=True)
            try:pins.rmdir()
            except OSError:pass
        return document
    finally:
        lock.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--topic", action="append", default=[])
    parser.add_argument("--finalize", action="store_true", help="Finalize a supervised recording receipt")
    args = parser.parse_args()
    result = finalize_recording(args.bag) if args.finalize else audit_bag(args.bag, args.topic)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if (result.get("state") == "complete" if args.finalize else result["complete"]) else 2


if __name__ == "__main__":
    raise SystemExit(main())
