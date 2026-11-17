"""Route topology and failure tests; coordinates are synthetic, no field truth."""
import copy
import json
import math
import numpy as np
import pytest
from nav_app.journey import grid_path, JourneyNetwork, JourneySession, grid_fingerprint
from nav_app.map_processing import GridMap, save_grid_map
from nav_app.geo_maps import GeoMaps, file_hash
from nav_app.outdoor import OutdoorState


def network(tmp_path):
    cloud=tmp_path/'map.pcd';cloud.write_bytes(b'map identity fixture')
    grid=GridMap(np.zeros((60,60),dtype=np.int8),.1,np.zeros(3))
    grid.occupancy[:,30]=100;grid.occupancy[20:40,30]=0
    path=tmp_path/'grid.yaml';save_grid_map(path,grid)
    scene=tmp_path/'room.scene.json'
    scene.write_text(json.dumps(dict(version=1,title='room',grid=path.name,cloud=cloud.name,coordinates_verified=True)))
    maps=GeoMaps(tmp_path)
    m=dict(schema=1,id='a'*32,title='room',saved_at=1.,cloud_path=str(cloud),sha256=file_hash(cloud),
           reference_source='local_only',scene_path=str(scene),grid_sha256=grid_fingerprint(path))
    maps.add(m)
    net=JourneyNetwork(tmp_path,maps)
    places=[dict(id='road-start',name='road start',domain='BD09',point=[116.30,39.9]),
            dict(id='road-door',name='door outside',domain='BD09',point=[116.3001,39.9]),
            dict(id='map-door',name='door inside',domain=m['id'],map_sha256=m['sha256'],point=[1.,3.]),
            dict(id='room',name='room',domain=m['id'],map_sha256=m['sha256'],point=[5.,3.])]
    route=dict(title='road',source='manual',coordinate_system='BD09',points=[p['point'] for p in places[:2]])
    net.replace(dict(schema=1,places=places,links=[
        dict(kind='road',**{'from':'road-start','to':'road-door'},route=route,bidirectional=True),
        dict(kind='portal',**{'from':'road-door','to':'map-door'},name='door',passage_confirmed=True,bidirectional=True)]))
    return net,m,path


def test_ending_journey_discards_pending_map_switch(tmp_path):
    state=OutdoorState(tmp_path)
    state.requests.extend([dict(action='journey_switch',id='old'), 'start'])
    state.journey_command({'action':'stop'})
    assert list(state.requests)==['start']


def test_road_portal_room_and_reverse_plan(tmp_path):
    net,m,_=network(tmp_path)
    plan=net.plan('road-start','room')
    assert [s['kind'] for s in plan['segments']]==['road','portal','local']
    assert 11<plan['distance_m']<15
    assert plan['bindings'][m['id']]['map_sha256']==m['sha256']
    reverse=net.plan('room','road-start')
    assert [s['kind'] for s in reverse['segments']]==['local','portal','road']
    assert reverse['segments'][1]['points']==[[1.,3.]]


@pytest.mark.parametrize('blocked',[-1,100])
def test_unknown_and_obstacles_do_not_connect_rooms(tmp_path,blocked):
    net,m,path=network(tmp_path)
    grid=GridMap(np.zeros((60,60),np.int8),.1,np.zeros(3));grid.occupancy[:,30]=blocked
    save_grid_map(path,grid);m['grid_sha256']=grid_fingerprint(path);net.maps.add(m)
    with pytest.raises(ValueError,match='连接'):net.plan('road-start','room')


def test_grid_and_cloud_changes_invalidate_plan(tmp_path):
    net,m,path=network(tmp_path)
    with path.open('a') as f:f.write('\n#changed\n')
    with pytest.raises(ValueError,match='变化'):net.plan('road-start','room')
    from pathlib import Path
    Path(m['cloud_path']).write_bytes(b'other map')
    with pytest.raises(ValueError,match='变化'):net.plan('road-start','room')


def test_wrong_map_id_unconfirmed_portal_and_disconnected_graph(tmp_path):
    net,_,_=network(tmp_path)
    for mutate in (lambda x:x['places'][2].update(map_sha256='b'*64),
                   lambda x:x['links'][1].update(passage_confirmed=False)):
        bad=copy.deepcopy(net.data);mutate(bad)
        with pytest.raises(ValueError):net.replace(bad)
    data=copy.deepcopy(net.data);data['links']=data['links'][:1];net.replace(data)
    with pytest.raises(ValueError,match='连接'):net.plan('road-start','room')


def local(m,p,t):
    return dict(local_pose=dict(map_id=m['id'],map_sha256=m['sha256'],position_m=[*p,0],stamp=t,heading_rad=0.))


def test_session_handoff_loss_recovery_and_restart(tmp_path):
    net,m,_=network(tmp_path);s=JourneySession(tmp_path);s.preview(net.plan('road-start','room'))
    road=net.data['places'][0]['point'];geo=dict(geographic_pose=dict(source='gnss'))
    def at(p,t):return dict(point=p,stamp=t)
    with pytest.raises(ValueError):s.begin({},None,100)
    s.begin(geo,at(road,100),100)
    end=net.data['places'][1]['point']
    for i in range(1,12):
        p=[road[0]+(end[0]-road[0])*i/11,road[1]]
        s.update(geo,at(p,100+i),100+i)
    assert s.phase=='transition' and s.segment['kind']=='portal'
    with pytest.raises(ValueError):s.confirm_transition({},None,120)
    with pytest.raises(ValueError):s.confirm_transition(local(m,[5,3],120),None,120)
    s.confirm_transition(local(m,[1,3],120),None,120)
    assert s.phase=='guiding' and s.segment['kind']=='local'
    s.update({},None,121);assert s.phase=='lost'
    for i in range(5):s.update(local(m,[1+i,3],122+i),None,122+i)
    s.update(local(m,[5,3],127),None,127)
    assert s.phase=='arrived'
    new=JourneySession(tmp_path);assert not new.active and new.plan is None


def test_self_crossings_stale_pose_and_teleport_cannot_skip_route(tmp_path):
    net,m,_=network(tmp_path);s=JourneySession();s.preview(net.plan('map-door','room'))
    s.begin(local(m,[1,3],100),None,100)
    s.update(local(m,[5,3],100.1),None,100.1)
    assert s.phase!='arrived' and s.progress<2
    s.update(local(m,[1,3],100),None,102);assert s.phase=='lost'
    wrong=local(m,[1,3],103);wrong['local_pose']['map_sha256']='b'*64
    s.update(wrong,None,103);assert s.phase=='lost'


def test_rotated_grid_and_footprint_no_corner_cut():
    grid=GridMap(np.zeros((30,30),np.int8),.1,np.array([5.,5.,math.pi/2]))
    points=grid_path(grid,[4.,6.],[3.,7.]);assert points[0]==[4.,6.] and points[-1]==[3.,7.]
    grid.occupancy[:,:]=100
    for i in range(30):grid.occupancy[i,i]=0
    with pytest.raises(ValueError):grid_path(grid,[4.,6.],[3.,7.])


def test_commands_share_single_session_and_reject_simulation(tmp_path):
    state=OutdoorState(tmp_path,clock=lambda:100.)
    state.simulation=object()
    with pytest.raises(ValueError):state.journey_command(dict(action='place',name='a',domain='BD09',point=[116.3,39.9]))
    state.simulation=None
    state.journey.phase='guiding'
    with pytest.raises(ValueError):state.simulate('enter')
    with pytest.raises(ValueError):state.control('start')
    with pytest.raises(ValueError):state.select_geo_map(None)
    state.journey_command(dict(action='stop'));assert not state.journey.active
