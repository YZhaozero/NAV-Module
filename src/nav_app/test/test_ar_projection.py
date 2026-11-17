import time
from types import SimpleNamespace as NS
import numpy as np
import pytest
from nav_app.ar_projection import project_camera, projected_route
from nav_app.nav_math import RenderStyle


def camera_node():
    n=NS(require_localizer_quality=True,localization_tools=NS(health=NS(valid=lambda _:True)),
        navigation_active=True,camera_error='',camera_calibration_error='',path_error='',
        walking=NS(phase='guiding'),operating_mode='walking',
        image_stamp=NS(sec=100,nanosec=0),_image_received_at=time.monotonic(),
        path=np.array([[0.,0.,2.],[0.,0.,4.],[0.,0.,6.]]),style=RenderStyle(),
        projection_mode='calibrated',camera_matrix=np.array([[400.,0,319.5],[0,400.,239.5],[0,0,1]]),
        camera_info_size=(640,480),camera_frame='camera',base_frame='body')
    n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_100_000_000))
    n.calls=[];n.transform=np.eye(4)
    def lookup(target,source,stamp):
        n.calls.append((target,source,stamp));return n.transform
    n.lookup_matrix=lookup
    n._grounded_route=lambda stamp:(n.path,'map',np.ones(3,dtype=bool))
    return n


def test_turning_changes_overlay_at_image_time_and_scaled_pixel_center():
    n=camera_node();a=projected_route(n,(480,640,3))
    assert a is not None and n.ar_status['code']=='ready'
    angle=.3;c,s=np.cos(angle),np.sin(angle)
    n.transform[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]]
    b=projected_route(n,(480,640,3))
    assert np.all(b[0][:,0]>a[0][:,0]+100)
    assert all(call[2] is n.image_stamp for call in n.calls)
    small=projected_route(n,(240,320,3))
    np.testing.assert_allclose(small[0],(b[0]+.5)/2-.5)


def test_distortion_matches_independent_opencv_projection():
    import cv2
    points=np.array([[.2,.1,1.],[-.4,.3,2.],[1.,-.7,3.]])
    K=camera_node().camera_matrix
    d=np.array([.15,-.04,.01,-.005,.02,.03,0,0])
    expected,_=cv2.projectPoints(points,np.zeros(3),np.zeros(3),K,d)
    actual,valid=project_camera(points,K,d,'rational_polynomial')
    assert valid.all();np.testing.assert_allclose(actual,expected[:,0],atol=1e-9)


@pytest.mark.parametrize('age',[-1,2])
def test_recently_received_but_old_or_future_camera_frame_is_hidden(age):
    n=camera_node();n.image_stamp.sec-=age
    assert projected_route(n,(480,640,3)) is None
    assert n.ar_status['code']=='camera_stale'


def test_missing_transform_and_invalid_localization_have_distinct_reasons():
    n=camera_node();n.lookup_matrix=lambda *_:None
    assert projected_route(n,(480,640,3)) is None and n.ar_status['code']=='camera_pose'
    n.localization_tools.health.valid=lambda _:False
    assert projected_route(n,(480,640,3)) is None and n.ar_status['code']=='localization'


def test_unsupported_lens_does_not_silently_draw_a_fake_route():
    n=camera_node();n.camera_distortion_model='equidistant'
    assert projected_route(n,(480,640,3)) is None and n.ar_status['code']=='calibration'


def test_handheld_tilt_does_not_tilt_the_flat_floor_fallback():
    from nav_app.ar_projection import grounded_route
    n=NS(ground_frame='missing_floor',base_frame='body',map_frame='map',path_frame='map',
         path=np.array([[2.,0.,10.],[4.,0.,-10.]]),style=RenderStyle(),ground_height_m=.8,ground_render_offset_m=.015)
    for angle in (-.5,0.,.5):
        c,s=np.cos(angle),np.sin(angle)
        body=np.eye(4);body[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]];body[2,3]=.8
        n.lookup_matrix=lambda target,source,stamp:None if target=='missing_floor' else body if source=='body' else np.eye(4)
        points,frame,visible=grounded_route(n,object())
        assert frame=='map' and visible.all()
        np.testing.assert_allclose(points[:,2],[.015,.015])


def test_route_in_front_but_outside_picture_is_not_reported_as_visible():
    n=camera_node();n.transform[0,3]=100.
    assert projected_route(n,(480,640,3)) is None
    assert n.ar_status['code']=='outside_view'


def test_route_crossing_image_is_visible_even_when_endpoints_are_outside():
    from nav_app.ar_projection import route_intersects_image
    valid=np.array([True,True])
    assert route_intersects_image(np.array([[-100.,200.],[900.,200.]]),valid,(480,640,3))
    assert not route_intersects_image(np.array([[-100.,100.],[100.,-300.]]),valid,(480,640,3))
    assert not route_intersects_image(np.array([[-100.,600.],[900.,600.]]),valid,(480,640,3))
