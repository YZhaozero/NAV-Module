"""Synthetic tests only; no claims about field GNSS accuracy."""
import json
import math
import struct
import time
import zipfile
from pathlib import Path

import numpy as np
import pytest
from nav_app.geodesy import GeoOrigin
from nav_app.features.trajectory_test import (
    TrajectoryTest, pairs_for, fit, analyze, load_session, export_session, tracks_for, valid_map_quality)
from nav_app.features.field_performance import diagnose, header_stamp


def samples(n=40):
    geo=GeoOrigin(121.,31.,0.)
    gnss=[]; odom=[]
    for i in range(n):
        lon,lat=geo.wgs84(i,0,0)
        gnss.append(dict(stamp=1000.+i,app_received_at=1000.+i+.1,received_at=1000.+i+.09,
                         timestamp_source='receiver_utc',valid_position=True,fix_quality=1,
                         longitude=lon,latitude=lat))
        odom.append(dict(stamp=1000.+i,app_received_at=1000.+i+3.1,source='fast_livo',frame='odom',child_frame='body',
                         position=[10.,20.+i,0.],quaternion=[0,0,0,1],epoch=0))
    config=dict(source='fast_livo',origin=[121.,31.,0.],max_pair_gap_s=.25)
    return gnss,odom,config


def test_raw_position_and_time_gates():
    g,o,c=samples()
    g[0]['valid_position']=False
    g[1]['timestamp_source']='unavailable'
    g[2]['stamp']=None
    g[4]['stamp']=g[3]['stamp']
    result=pairs_for(g,o,c)
    assert [p['reason'] for p in result[:5]] == ['invalid_position','receiver_time_unavailable','receiver_time_unavailable','paired','duplicate_or_backwards_time']
    assert result[5]['odom_age_s'] == pytest.approx(3.1)


def test_interpolation_is_bounded_and_lever_arm_frame_checked():
    g,o,c=samples(1)
    a=dict(o[0],stamp=999.9); b=dict(o[0],stamp=1000.1,position=[12,20,0])
    result=pairs_for(g,[a,b],c)[0]
    assert result['paired'] and result['odom'][0] == pytest.approx(11)
    assert pairs_for(g,[dict(a,stamp=999),b],c)[0]['reason']=='gap_or_epoch'
    assert pairs_for(g,[dict(a,epoch=1),b],c)[0]['reason']=='gap_or_epoch'
    c.update(antenna_in_body_m=[1,0,0],odom_child_frame='base_link')
    assert pairs_for(g,[a,b],c)[0]['reason']=='lever_arm_frame_mismatch'


def test_frozen_alignment_excludes_calibration_and_retains_drift():
    g,o,c=samples()
    a=fit(pairs_for(g[:20],o,c)); c['alignment']=a
    assert a['yaw']==pytest.approx(math.pi/2,abs=1e-5)
    for item in o[20:]:item['position'][0]+=4
    paired,report=analyze(g,o,c)
    assert report['evaluation_count']==20
    assert report['p95_m']==pytest.approx(4,abs=.001)
    assert paired[0]['difference_m'] < 1e-8
    assert c['alignment']==a


def test_insufficient_calibration_is_rejected():
    g,o,c=samples(5)
    with pytest.raises(ValueError):fit(pairs_for(g,o,c))


def test_record_replay_export_and_reset(tmp_path):
    recorder=TrajectoryTest(tmp_path)
    try:
        path=Path(recorder.call('start',{'source':'fast_livo'}))
        with pytest.raises(ValueError):export_session(path)
        g,o,c=samples()
        for i in range(20):
            recorder.feed('gnss',g[i]); recorder.feed('odometry',o[i])
        alignment=recorder.call('align')
        with pytest.raises(ValueError):recorder.call('align')
        with pytest.raises(ValueError):recorder.call('start',{})
        assert recorder.active  # repeated start must not close the active files
        for i in range(20,40):
            recorder.feed('gnss',g[i]); recorder.feed('odometry',o[i])
        report=recorder.call('stop')
        assert report['total']==40 and report['evaluation_count']==20
        data=load_session(path)
        assert len(data['odometry'])==40 and data['manifest']['alignment']==alignment
        assert data['gnss'][0]['fix_quality']==1
        assert (path/'pairs.csv').is_file() and (path/'viewer.html').is_file()
        target=export_session(path)
        with zipfile.ZipFile(target) as z:assert path.name+'/performance.jsonl' in z.namelist()
        recorder.call('start',{})
        recorder.feed('odometry',o[0]); recorder.feed('odometry',dict(o[0],stamp=900))
        recorder.call('stop')
        assert recorder.manifest['epoch_changed']
        assert recorder.manifest['alignment'] is None
    finally:recorder.close()


def test_bounded_queue_reports_missing_and_shutdown_flush(tmp_path):
    recorder=TrajectoryTest(tmp_path)
    recorder.call('start',{})
    recorder.queued_bytes=8*1024**2
    recorder.feed('events',{'event':'overflow'})
    assert recorder.dropped==1
    recorder.queued_bytes=0
    recorder.close()
    m=json.loads((recorder.session/'manifest.json').read_text())
    assert m['dropped']==1 and m['state']=='incomplete'


def test_quality_and_three_track_frames():
    g,o,c=samples(); c.update(alignment=fit(pairs_for(g[:20],o,c)),map_sha256='abc',map_frame='map')
    q=dict(valid=True,map_sha256='abc',map_frame='map',odom_frame='odom',odom_source='fast_livo',data_age_s=.1,
           stamp=1000.,app_received_at=1000.1,last_success_stamp_ns=1000*10**9,
           map_from_odom=dict(rotation_xyzw=[0,0,0,1],translation_m=[100,0,0]))
    assert valid_map_quality(q,c)
    assert not valid_map_quality(dict(q,map_sha256='wrong'),c)
    assert not valid_map_quality(dict(q,odom_source='dlio'),c)
    assert not valid_map_quality(dict(q,app_received_at=1004),c)
    c['map_from_odom']=q['map_from_odom']
    tracks,report,frame=tracks_for(dict(gnss=g,odometry=o,localization=[q],manifest=c))
    assert frame=='map' and tracks['odometry'][0][0]==110
    assert tracks['gnss'][0][0]==pytest.approx(110)
    c['epoch_changed']=True
    assert tracks_for(dict(gnss=g,odometry=o,localization=[q],manifest=c))[2]=='odom'


def telemetry(age=3.1,busy=.95,backlog=2.8,raw_age=.1,wait='ready',offset=.1):
    return [dict(received_monotonic=float(i), data=dict(instance='one',state_age_s=age,busy_ratio=busy,
            lidar_backlog_s=backlog,latest_lidar_s=1000.,latest_imu_s=1000.+offset,wait_reason=wait),
            raw={k:dict(received_monotonic=float(i),age_s=raw_age) for k in ('lidar','imu')}) for i in range(1,11)]


@pytest.mark.parametrize('changes,expected',[
    ({},'compute_backlog'), ({'raw_age':3.1},'input_time'),
    ({'busy':.1,'backlog':0,'wait':'imu_behind','offset':1},'sync_wait'),
    ({'busy':.1,'backlog':0},'uncertain'), ({'age':.15,'busy':.1,'backlog':0},'fresh')])
def test_causal_diagnosis(changes,expected):
    assert diagnose(telemetry(**changes),10)['code']==expected


def test_suspended_and_transport_wait_override_fresh_or_compute_diagnosis():
    data=telemetry(age=.15)
    data[-1]['data']['tracking_suspended']=True
    assert diagnose(data[-1:],10)['code']=='tracking_suspended'
    data[-1]['data']['tracking_suspended']=False
    data[-1]['data']['pose_output_pending']=100
    assert diagnose(data,10)['code']=='pose_output_wait'


def test_missing_stalled_and_new_instance_are_not_healthy():
    data=telemetry()
    assert diagnose(data,15)['code']=='no_telemetry'
    data[-1]['raw']={}
    assert diagnose(data,10)['code']=='missing_input'
    data[-1]['data']['instance']='new'
    assert diagnose(data,10)['code']=='warming'


def test_serialized_sensor_header():
    for prefix,enc in (('<',1),('>',0)):
        payload=bytes([0,enc,0,0])+struct.pack(prefix+'iI',1000,250000000)
        assert header_stamp(payload)==1000.25
    with pytest.raises(ValueError):header_stamp(b'bad')


def test_truncated_record_retains_evidence(tmp_path):
    recorder=TrajectoryTest(tmp_path); recorder.call('start',{}); recorder.close()
    with (recorder.session/'gnss.jsonl').open('a') as f:f.write('{"truncated":')
    assert load_session(recorder.session)['incomplete_lines']==1


def test_map_snapshot_identity_and_replay(tmp_path):
    from nav_app.geo_maps import file_hash
    from nav_app.map_processing import GridMap, save_grid_map, save_pcd
    occupancy = np.zeros((12, 20), dtype=np.int8)
    grid = GridMap(occupancy, .05, np.array([0., 0., 0.]), frame_id='map')
    grid_path, _ = save_grid_map(tmp_path/'demo.yaml', grid)
    cloud_path = save_pcd(tmp_path/'demo.pcd', np.array([
        [0., 0., 0.], [2., 0., .3], [2., 6., .8], [0., 6., .2]],
        dtype=np.float32))
    recorder=TrajectoryTest(tmp_path/'tests')
    config=dict(source='fast_livo',context='map',cloud_path=str(cloud_path),grid_path=str(grid_path),
                map_sha256=file_hash(cloud_path),map_frame='map')
    try:
        path=Path(recorder.call('start',config))
        recorder.call('stop')
        with np.load(path/'map-preview.npz',allow_pickle=False) as saved:
            assert np.array_equal(saved['occupancy'],grid.occupancy)
            assert len(saved['cloud'])>0
        with zipfile.ZipFile(export_session(path)) as archive:
            assert path.name+'/map-preview.npz' in archive.namelist()
        with pytest.raises(ValueError):recorder.call('start',dict(config,map_sha256='wrong'))
        assert not recorder.active and recorder.manifest['state']=='incomplete'
        path=Path(recorder.call('start',dict(source='fast_livo',context='mapping')))
        g,o,c=samples(1); recorder.feed('odometry',o[0])
        cloud=np.array([[0.,0.,0.],[1.,1.,1.]])
        assert not recorder.call('preview',dict(cloud=cloud,frame='body'))
        assert recorder.call('preview',dict(cloud=cloud,frame='odom'))
        recorder.call('stop')
        assert (path/'map-preview.npz').is_file()
    finally:recorder.close()


def test_interrupted_export_generates_analysis_without_rewriting_source(tmp_path):
    recorder=TrajectoryTest(tmp_path); path=Path(recorder.call('start',{})); recorder.close()
    manifest=json.loads((path/'manifest.json').read_text()); manifest['state']='recording'
    (path/'manifest.json').write_text(json.dumps(manifest))
    with zipfile.ZipFile(export_session(path)) as archive:
        copied=json.loads(archive.read(path.name+'/manifest.json'))
        assert copied['state']=='interrupted'
        assert path.name+'/viewer.html' in archive.namelist()
    assert json.loads((path/'manifest.json').read_text())['state']=='recording'


def test_symlink_session_metadata_is_refused(tmp_path):
    recorder=TrajectoryTest(tmp_path); path=Path(recorder.call('start',{})); recorder.close()
    metadata=path/'manifest.json'; target=tmp_path/'other.json'; metadata.rename(target); metadata.symlink_to(target)
    with pytest.raises(ValueError):load_session(path)
