"""Durable close is distinct from offline readback and its cancellation."""
import json
import struct
from pathlib import Path

import pytest
import yaml

from nav_app.features.bag_audit import finalize_recording


def recording(root, *, profile='full', errors=()):
    root.mkdir(exist_ok=True)
    (root/'part.db3').write_bytes(b'closed split')
    info = dict(relative_file_paths=['part.db3'], storage_identifier='sqlite3', message_count=3,
        duration=dict(nanoseconds=12_000_000_000), topics_with_message_count=[dict(
            topic_metadata=dict(name='/imu', type='sensor_msgs/msg/Imu'), message_count=3)])
    (root/'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=info)))
    manifest = dict(state='stored', storage_state='stored', audit_state='pending',
        storage_flush_confirmed=True, selected_topics=['/imu'], storage_errors=list(errors),
        errors=list(errors), recording_profile=profile)
    if profile == 'keyframes':
        manifest['keyframes'] = dict(expected_topic_counts={'/imu':3}, statistics={'saved':3})
    (root/'nav_manifest.json').write_text(json.dumps(manifest))
    return manifest


def reader_factory(root, info):
    class Reader:
        rows = [('/imu', b'\x00\x01\x00\x00'+struct.pack('<iI', 1, n)+b'\0'*12,
                 1_000_000_000+n) for n in (0, 10_000_000, 20_000_000)]
        def has_next(self): return bool(self.rows)
        def read_next(self): return self.rows.pop(0)
    return Reader()


def test_stored_receipt_is_not_complete_until_offline_readback(tmp_path):
    recording(tmp_path)
    progress=[]
    result=finalize_recording(tmp_path, progress=lambda done,total:progress.append((done,total)),
                              reader_factory=reader_factory)
    assert result['state']=='complete' and result['audit']['complete']
    assert result['storage_state']=='stored' and result['audit_state']=='complete'
    assert progress[0]==(0,3) and progress[-1]==(3,3)
    assert json.loads((tmp_path/'nav_manifest.json').read_text())==result
    # Retrying a completed task does not re-read the payload.
    assert finalize_recording(tmp_path,reader_factory=lambda *_:pytest.fail('duplicate audit'))==result


def test_capture_errors_are_retained_even_when_physical_readback_passes(tmp_path):
    recording(tmp_path, errors=['编码时缺少2帧'])
    result=finalize_recording(tmp_path,reader_factory=reader_factory)
    assert result['audit']['complete'] and result['state']=='incomplete'
    assert result['errors']==['编码时缺少2帧']


def test_cancellation_preserves_pending_receipt_and_never_publishes_incomplete(tmp_path):
    recording(tmp_path)
    before=(tmp_path/'nav_manifest.json').read_bytes()
    seen=[]
    with pytest.raises(InterruptedError):
        finalize_recording(tmp_path,progress=lambda done,total:seen.append(done),
                            cancelled=lambda:bool(seen),reader_factory=reader_factory)
    assert before==(tmp_path/'nav_manifest.json').read_bytes()
    assert not (tmp_path/'nav_audit.json').exists()
    assert finalize_recording(tmp_path,reader_factory=reader_factory)['state']=='complete'


def test_unclosed_recording_is_rejected_without_reading_or_changing_it(tmp_path):
    document=recording(tmp_path)
    document.update(state='flushing',storage_flush_confirmed=False)
    (tmp_path/'nav_manifest.json').write_text(json.dumps(document))
    with pytest.raises(ValueError,match='落盘'):
        finalize_recording(tmp_path,reader_factory=lambda *_:pytest.fail('read open writer'))
    assert json.loads((tmp_path/'nav_manifest.json').read_text())==document


def test_changed_split_cannot_be_committed_as_the_audited_source(tmp_path):
    recording(tmp_path)
    def changed(root, info):
        (root/'part.db3').write_bytes(b'different payload')
        return reader_factory(root,info)
    with pytest.raises(ValueError,match='发生变化'):
        finalize_recording(tmp_path,reader_factory=changed)
    assert json.loads((tmp_path/'nav_manifest.json').read_text())['state']=='stored'


def test_sparse_keyframes_use_counts_not_raw_stream_continuity(tmp_path):
    recording(tmp_path,profile='keyframes')
    result=finalize_recording(tmp_path,reader_factory=reader_factory)
    assert result['state']=='complete' and result['audit']['intentional_sampling']
    assert result['audit']['topic_counts']=={'/imu':3}


def test_sparse_write_count_mismatch_is_not_hidden_by_matching_metadata(tmp_path):
    document=recording(tmp_path,profile='keyframes')
    document['keyframes']['expected_topic_counts']['/imu']=4
    (tmp_path/'nav_manifest.json').write_text(json.dumps(document))
    result=finalize_recording(tmp_path,reader_factory=reader_factory)
    assert result['state']=='incomplete' and '数量' in result['errors'][0]




def test_recorder_reports_stored_metadata_without_calling_it_complete(tmp_path):
    from PyQt5.QtWidgets import QApplication
    from nav_app.features.rosbag import RosbagController
    app=QApplication.instance() or QApplication([])
    recording(tmp_path)
    controller=RosbagController();controller._record_output=tmp_path
    done=[];controller.record_finished.connect(done.append)
    controller._record_state('stopped')
    assert len(done)==1 and done[0].message_count==3
    assert controller.last_record_state=='stored'


def test_record_controller_admission_and_stop_guard_apply_at_launch(tmp_path,monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from nav_app.features.rosbag import RosbagController, PlaybackRequest
    app=QApplication.instance() or QApplication([])
    controller=RosbagController();events=[]
    controller.admission_guard=lambda kind:(False,'地图任务尚未退出')
    monkeypatch.setattr(controller.record_process,'start_launch',lambda *_:events.append('start'))
    assert controller.start_recording(tmp_path,'new',['/imu'])==(False,'地图任务尚未退出')
    assert controller.start_playback(PlaybackRequest(tmp_path,('/imu',)))==(False,'地图任务尚未退出')
    controller.stop_guard=lambda kind:(False,'请从当前行程保存结束')
    monkeypatch.setattr(controller.record_process,'stop_launch',lambda:events.append('stop'))
    controller.stop_recording()
    assert events==[]


def test_compression_cancellation_keeps_original_and_metadata(tmp_path):
    from nav_app.features.bag_compression import compress_closed_bag
    source=tmp_path/'part.db3';source.write_bytes(b'long data'*100_000)
    path=tmp_path/'metadata.yaml'
    path.write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=dict(relative_file_paths=['part.db3']))))
    before=path.read_bytes();calls=[]
    def cancelled():
        calls.append(1);return len(calls)>=3
    with pytest.raises(InterruptedError):compress_closed_bag(tmp_path,cancelled=cancelled)
    assert source.exists() and path.read_bytes()==before
    assert not list(tmp_path.glob('*.partial'))
    assert compress_closed_bag(tmp_path)['verified_splits'][0]['raw_bytes']==900_000


def test_diagnostic_snapshot_pins_rotated_log_and_excludes_later_capture(tmp_path):
    from nav_app.features.capture_evidence import snapshot_evidence, collect_evidence
    source=tmp_path/'logs';source.mkdir()
    trace=source/'sensor.log';trace.write_bytes(b'first capture\n')
    output=tmp_path/'bag';output.mkdir()
    snapshot=snapshot_evidence(output,0,session=source)
    assert len(snapshot['files'])==1 and 'pin' in snapshot['files'][0]
    trace.rename(source/'rotated.old')
    trace.write_bytes(b'new capture must not enter old evidence\n')
    with (source/'rotated.old').open('ab') as stream:stream.write(b'later append\n')
    result=collect_evidence(output,0,snapshot=snapshot)
    assert not result['errors'] and result['files'][0]['bytes']==len(b'first capture\n')
    assert (output/'diagnostics/application/sensor.log').read_bytes()==b'first capture\n'


def test_diagnostic_copy_cancel_keeps_pins_and_can_retry(tmp_path):
    from nav_app.features.capture_evidence import snapshot_evidence, collect_evidence
    source=tmp_path/'logs';source.mkdir()
    (source/'sensor.log').write_bytes(b'large trace\n'*20_000)
    output=tmp_path/'bag';output.mkdir()
    snapshot=snapshot_evidence(output,0,session=source)
    calls=[]
    def cancelled():calls.append(1);return len(calls)>=3
    with pytest.raises(InterruptedError):collect_evidence(output,0,snapshot=snapshot,cancelled=cancelled)
    assert Path(snapshot['files'][0]['pin']).exists()
    assert not list((output/'diagnostics').rglob('.archive-*'))
    assert not collect_evidence(output,0,snapshot=snapshot)['errors']


def test_finalizer_archives_frozen_diagnostics_then_releases_pins(tmp_path):
    from nav_app.features.capture_evidence import snapshot_evidence
    logs=tmp_path/'logs';logs.mkdir();(logs/'sensor.log').write_text('capture ended\n')
    bag=tmp_path/'bag';document=recording(bag)
    snapshot=snapshot_evidence(bag,0,session=logs)
    document.update(diagnostic_snapshot=snapshot, diagnostics_state='pending',diagnostics_complete=False)
    (bag/'nav_manifest.json').write_text(json.dumps(document))
    stages=[]
    result=finalize_recording(bag,reader_factory=reader_factory,
        stage_progress=lambda phase,done,total:stages.append((phase,done,total)))
    assert result['diagnostics_complete'] and result['diagnostics_state']=='complete'
    assert not (bag/'.diagnostic-snapshot').exists()
    assert (bag/'diagnostics/application/sensor.log').read_text()=='capture ended\n'
    assert stages[0]==('audit',0,3) and stages[-1]==('evidence',1,1)


@pytest.mark.parametrize('storage_id',['sqlite3','mcap'])
def test_actual_closed_rosbag_readback_without_ros_node(tmp_path,storage_id):
    rosbag2_py=pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu
    bag=tmp_path/'real'
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=storage_id),
                rosbag2_py.ConverterOptions('',''))
    writer.create_topic(rosbag2_py.TopicMetadata(name='/imu',type='sensor_msgs/msg/Imu',serialization_format='cdr'))
    for i in range(40):
        message=Imu();message.header.stamp.sec=1;message.header.stamp.nanosec=i*5_000_000
        writer.write('/imu',serialize_message(message),1_000_000_000+i*5_000_000)
    del writer
    document=dict(state='stored',storage_flush_confirmed=True,storage_state='stored',audit_state='pending',
                  selected_topics=['/imu'],errors=[],storage_errors=[])
    (bag/'nav_manifest.json').write_text(json.dumps(document))
    result=finalize_recording(bag)
    assert result['state']=='complete' and result['audit']['topics']['/imu']['count']==40


def test_completed_audit_survives_later_evidence_cancellation(tmp_path):
    from nav_app.features.capture_evidence import snapshot_evidence
    logs=tmp_path/'logs';logs.mkdir();(logs/'sensor.log').write_text('closed trace')
    bag=tmp_path/'bag';document=recording(bag)
    document['diagnostic_snapshot']=snapshot_evidence(bag,0,session=logs)
    (bag/'nav_manifest.json').write_text(json.dumps(document))
    phases=[]
    with pytest.raises(InterruptedError):
        finalize_recording(bag,reader_factory=reader_factory,
            stage_progress=lambda phase,*_:phases.append(phase),cancelled=lambda:'evidence' in phases)
    assert json.loads((bag/'nav_audit.json').read_text())['complete']
    assert json.loads((bag/'nav_manifest.json').read_text())['state']=='stored'
    result=finalize_recording(bag,reader_factory=lambda *_:pytest.fail('completed readback repeated'))
    assert result['state']=='complete' and result['diagnostics_complete']


def test_completed_evidence_survives_later_compression_cancellation(tmp_path,monkeypatch):
    from nav_app.features.capture_evidence import snapshot_evidence
    from nav_app.features import bag_compression, capture_evidence
    logs=tmp_path/'logs';logs.mkdir();(logs/'sensor.log').write_text('closed trace')
    bag=tmp_path/'bag';document=recording(bag)
    document.update(diagnostic_snapshot=snapshot_evidence(bag,0,session=logs),
                    buffer_budget={'post_record_compression':'zstd'})
    (bag/'nav_manifest.json').write_text(json.dumps(document))
    compress=bag_compression.compress_closed_bag
    def interrupted(*_args,**_kwargs):raise InterruptedError('switch to capture')
    monkeypatch.setattr(bag_compression,'compress_closed_bag',interrupted)
    with pytest.raises(InterruptedError):finalize_recording(bag,reader_factory=reader_factory)
    assert json.loads((bag/'nav_diagnostics.json').read_text())['_snapshot_fingerprint']
    monkeypatch.setattr(bag_compression,'compress_closed_bag',compress)
    monkeypatch.setattr(capture_evidence,'collect_evidence',lambda *_a,**_k:pytest.fail('completed evidence repeated'))
    result=finalize_recording(bag,reader_factory=lambda *_:pytest.fail('completed readback repeated'))
    assert result['state']=='complete' and result['final_compression']['verified_splits']
