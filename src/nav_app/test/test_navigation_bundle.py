import json
import numpy as np
import pytest
from nav_app.features.navigation_bundle import build,observed_floor_grid,estimate_floor
from nav_app.map_processing import save_pcd,load_grid_map,read_pgm
from nav_app.scene_catalog import read_scene


def measured_cloud():
    x,y=np.meshgrid(np.linspace(0,3,61),np.linspace(0,2,41))
    floor=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    wall=np.array([[2.,y,z] for y in np.linspace(0,1,21) for z in (.2,.5,1.)])
    return np.r_[floor,wall]


def test_observed_floor_obstacles_and_unknown_remain_three_distinct_states():
    grid=observed_floor_grid(measured_cloud(),0.)
    assert set(np.unique(grid.occupancy))=={-1,0,100}
    def cell(x,y):
        ix,iy=np.floor((np.array([x,y])-grid.origin[:2])/grid.resolution).astype(int)
        return grid.occupancy[iy,ix]
    assert cell(.5,.5)==0 and cell(2.,.5)==100 and cell(-.3,-.3)==-1
    # An empty observed floor with no obstacle returns is still usable.
    floor=measured_cloud();floor=floor[floor[:,2]==0]
    assert set(np.unique(observed_floor_grid(floor,0.).occupancy))=={-1,0}
    with pytest.raises(ValueError,match='观测不足'):observed_floor_grid(measured_cloud(),10.)


def test_bundle_auto_links_same_coordinates_without_overwriting_sources(tmp_path,monkeypatch):
    monkeypatch.setenv('NAV_RUNTIME_DIR',str(tmp_path/'runtime'))
    cloud=tmp_path/'source.pcd';save_pcd(cloud,measured_cloud())
    original=cloud.read_bytes()
    def no_index(*_):raise ValueError('not installed on test host')
    result=build(cloud,tmp_path/'maps','测试场地',0.,index_builder=no_index)
    scene=read_scene(result['scene'])
    assert scene.ready and not scene.global_ready and result['warning']
    assert cloud.read_bytes()==original
    grid=load_grid_map(scene.grid)
    assert set(np.unique(grid.occupancy))=={-1,0,100}
    assert set(np.unique(read_pgm(str(scene.grid).replace('.yaml','.pgm'))))=={0,205,254}
    data=json.loads(open(result['scene'],encoding='utf-8').read())
    assert data['generated']['coordinates']=='original_XYZ'
    assert data['generated']['field_validated'] is False


def test_large_map_adapts_grid_resolution_without_cropping_measured_points():
    points=np.r_[measured_cloud(),[[155.,135.,.5]]]
    grid=observed_floor_grid(points,0.)
    assert grid.resolution>.05 and grid.occupancy.size<=8_000_000
    ix,iy=np.floor((np.array([155.,135.])-grid.origin[:2])/grid.resolution).astype(int)
    assert grid.occupancy[iy,ix]==100
    assert set(np.unique(grid.occupancy))=={-1,0,100}


def test_automatic_floor_uses_spatial_support_and_keeps_original_axes(tmp_path, monkeypatch):
    monkeypatch.setenv('NAV_RUNTIME_DIR',str(tmp_path/'runtime'))
    points = measured_cloud(); points[:,2] -= 1.17
    # Dense high returns and a ceiling must not dominate ground detection.
    wall = np.repeat(points[points[:,2] > -1.], 100, axis=0)
    ceiling = points[points[:,2] == -1.17].copy(); ceiling[:,2] += 3.
    mixed = np.r_[points, wall, ceiling]
    assert estimate_floor(mixed) == pytest.approx(-1.17, abs=.001)
    source = tmp_path/'auto.pcd'; save_pcd(source, mixed)
    before = source.read_bytes()
    result = build(source, tmp_path/'maps', '自动地图', index_builder=lambda *_: '',gravity_aligned=True)
    assert result['floor_estimated'] and read_scene(result['scene']).ready
    assert source.read_bytes() == before
    assert read_scene(result['scene']).gravity_aligned


def test_sloping_observed_path_is_connected_but_wall_and_unknown_stay_blocked():
    from scipy.ndimage import label
    x,y=np.meshgrid(np.arange(0,10,.04),np.arange(0,3,.04))
    floor=np.c_[x.ravel(),y.ravel(),(-1.5-.035*x).ravel()]
    wall=np.array([[5.,y,-1.675+z] for y in np.arange(0,1,.04) for z in np.arange(.2,1.8,.08)])
    points=np.r_[floor,wall,[[2,1,-8.]]]
    before=points.copy()
    grid=observed_floor_grid(points,-1.5,follow_terrain=True)
    assert np.array_equal(points,before)
    def cell(x,y):
        i,j=np.floor((np.array([x,y])-grid.origin[:2])/grid.resolution).astype(int)
        return j,i
    components,_=label(grid.occupancy==0,np.ones((3,3)))
    assert components[cell(1,2)]>0
    assert components[cell(1,2)]==components[cell(9,2)]
    assert grid.occupancy[cell(5,.5)]==100
    assert grid.occupancy[cell(-.3,-.3)]==-1


def test_isolated_tabletop_is_not_treated_as_walkable_floor():
    x,y=np.meshgrid(np.arange(0,8,.04),np.arange(0,5,.04))
    table=(x>3)&(x<5)&(y>1.5)&(y<3.5)
    points=np.c_[x.ravel(),y.ravel(),np.where(table,.8,0.).ravel()]
    grid=observed_floor_grid(points,0.,follow_terrain=True)
    i,j=np.floor((np.array([4.,2.5])-grid.origin[:2])/grid.resolution).astype(int)
    assert grid.occupancy[j,i]!=0
