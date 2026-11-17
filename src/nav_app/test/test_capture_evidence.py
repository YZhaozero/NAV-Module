import hashlib
import json
import os
import time
from pathlib import Path
from nav_app.features.capture_evidence import bound_record_buffers, collect_evidence, recorder_fault
from nav_app.features.log_store import LogStore


def test_old_settings_have_effective_byte_budget():
    args=["bag","record","--max-cache-size","1000000000","--compression-mode","message",
          "--compression-queue-size","400","/camera/image"]
    result,budget=bound_record_buffers(args,2*1024**2)
    assert args[3]=="1000000000"
    assert budget["cache_bytes"]==128*1024**2
    assert budget["compression_queue_messages"]==0
    assert budget["post_record_compression"]=="zstd"
    assert "--compression-mode" not in result
    assert result[-1]=="/camera/image"
    assert budget["compression_queue_estimate_bytes"]<=64*1024**2


def test_humble_actual_queue_drop_warning_is_fatal():
    assert recorder_fault("[rosbag2_compression]: Dropping message on topic:/livox/imufrom compression queue because it is full. Queue size: 25")
    assert not recorder_fault("Recording stopped")
    assert recorder_fault("[rosbag2_cpp]: Cache buffers lost messages per topic:")


def test_mcap_uses_native_lossless_chunks_without_post_record_recompression(tmp_path):
    args=['bag','record','--storage','mcap','--max-cache-size','1000000000','--compression-mode','message','--compression-queue-size','100','/rgb','/imu']
    config=tmp_path/'storage.yaml'
    result,budget=bound_record_buffers(args,2764872,storage_config_path=config)
    assert result[result.index('--storage-config-file')+1]==str(config)
    assert '--storage-preset-profile' not in result
    assert '--compression-mode' not in result and '--compression-queue-size' not in result
    assert budget['post_record_compression'] is None
    assert budget['cache_bytes']==256*1024**2 and result[-2:]==['/rgb','/imu']


def test_stream_compression_roundtrip_and_truncation(tmp_path):
    from nav_app.features.bag_compression import transform,compress_closed_bag
    import pytest,yaml
    source=tmp_path/"test.db3"
    # Compressible plus incompressible data exercises repeated small output buffers.
    source.write_bytes(b"abcdefgh"*1000000+os.urandom(512*1024))
    expected=hashlib.sha256(source.read_bytes()).hexdigest()
    metadata={"rosbag2_bagfile_information":{"relative_file_paths":["test.db3"],"files":[{"path":"test.db3"}]}}
    (tmp_path/"metadata.yaml").write_text(yaml.safe_dump(metadata))
    progress=[]
    report=compress_closed_bag(tmp_path, progress=lambda done,total: progress.append((done,total)))
    assert progress==[(1,1)]
    assert report["verified_splits"][0]["raw_sha256"]==expected
    assert not source.exists()
    compressed=tmp_path/"test.db3.zstd"
    assert transform(compressed,decode=True)==expected
    changed=yaml.safe_load((tmp_path/"metadata.yaml").read_text())["rosbag2_bagfile_information"]
    assert changed["compression_mode"]=="FILE" and changed["files"][0]["path"]=="test.db3.zstd"
    with compressed.open("r+b") as stream:stream.truncate(compressed.stat().st_size-1)
    with pytest.raises(ValueError,match="截断"):transform(compressed,decode=True)


def test_stream_decompression_exact_output_block_boundary(tmp_path):
    from nav_app.features.bag_compression import transform,BLOCK
    source=tmp_path/"raw"; source.write_bytes(b"z"*(BLOCK*8))
    target=tmp_path/"compressed"
    assert transform(source,target)==transform(target,decode=True)


def test_evidence_snapshot_exact_bytes_drop_detection_and_archive(tmp_path):
    store=LogStore(tmp_path/"logs")
    store.write("mapping","mapping","startup failed then recovered")
    assert store.flush()
    trace=store.session/"sensors/evidence/livox-123-456"
    trace.mkdir(parents=True)
    payload=bytes(range(80))*10000
    (trace/"packets-000000.bin").write_bytes(payload+b"partial")
    (trace/"status.json").write_text(json.dumps(dict(dropped=3,closed=False)))
    result=collect_evidence(tmp_path/"bag",time.time(),session=store.session)
    packet=next(row for row in result["files"] if row["path"].endswith(".bin"))
    assert packet["bytes"]==len(payload)
    assert packet["sha256"]==hashlib.sha256(payload).hexdigest()
    assert any("诊断流有缺失" in error for error in result["errors"])
    assert any("mapping.log" in row["path"] for row in result["files"])
    store.finish()
    entry=store.entries()[0]
    assert entry["status"]=="completed"
    archive=store.export(entry)
    assert Path(archive).exists()


def test_evidence_ignores_symlinks_and_old_segments(tmp_path):
    source=tmp_path/"source"; source.mkdir()
    secret=tmp_path/"secret.json"; secret.write_text('"not evidence"')
    (source/"alias.json").symlink_to(secret)
    (source/"directory").symlink_to(tmp_path,target_is_directory=True)
    old=source/"old.log"; old.write_text("old")
    os.utime(old,(1,1))
    result=collect_evidence(tmp_path/"bag",time.time(),session=source)
    assert not result["files"]


def test_archived_rtk_bytes_are_counted_without_claiming_gnss_observations(tmp_path):
    import base64
    root = tmp_path/'session'
    trace = root/'sensors/evidence/rtk-123-456'
    trace.mkdir(parents=True)
    rows = [dict(event=kind, bytes=len(data), base64=base64.b64encode(data).decode())
            for kind, data in [('serial_rx', b'$GNGGA,0*00\r\n'), ('serial_tx', b'AT\r\n'),
                               ('ntrip_body_rx', b'\xd3\x00\x01')]]
    (trace/'events-000000.jsonl').write_text('\n'.join(json.dumps(row) for row in rows)+'\n')
    result = collect_evidence(tmp_path/'bag', time.time(), session=root)
    assert result['rtk_raw']['serial_rx_bytes'] == 13
    assert result['rtk_raw']['ntrip_body_rx_bytes'] == 3
    assert result['rtk_raw']['sessions'] == 1
    assert result['rtk_raw']['rover_observations_verified'] is False
    assert result['rtk_raw']['malformed_records'] == 0


def test_archived_rover_observations_are_distinguished_from_base_only(tmp_path):
    import base64
    from nav_rtk.parser.ntrip import crc24q
    # A complete MSM4 with one valid pseudorange/carrier-phase cell.
    bits=f'{1074:012b}'+f'{1:012b}'+'0'*49+'1'+'0'*63+'1'+'0'*31+'1'
    bits+=f'{70:08b}'+'0'*10+'0'*48
    bits+='0'*(-len(bits)%8)
    body=int(bits,2).to_bytes(len(bits)//8,'big')
    raw=b'\xd3'+len(body).to_bytes(2,'big')+body
    raw+=crc24q(raw).to_bytes(3,'big')
    trace=tmp_path/'session/sensors/evidence/rtk-123-456';trace.mkdir(parents=True)
    rows=[dict(event=event,bytes=len(raw),base64=base64.b64encode(raw).decode())
          for event in ('ntrip_body_rx','rover_rtcm')]
    (trace/'events-000000.jsonl').write_text('\n'.join(map(json.dumps,rows)))
    result=collect_evidence(tmp_path/'bag',time.time(),session=tmp_path/'session')['rtk_raw']
    assert result['rover_observations_verified'] and result['measurement_cells']==1
    assert result['rover_message_types']=={'1074':1}
    assert result['rover_rtcm_bytes']==len(raw) and not result['malformed_records']
