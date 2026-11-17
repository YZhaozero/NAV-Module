import http.client
import json
import math
from pathlib import Path
import tempfile
import unittest
from nav_app.outdoor import OutdoorState, GeoOrigin, fit_alignment, distance
from nav_app.phone_remote import RemoteState, RemoteServer
from nav_app.outdoor_service import MapServices


def route():
    return dict(title="验证路线",coordinate_system="BD09",source="manual",points=[[116.3,39.9],[116.301,39.9],[116.301,39.901]])


def fix(stamp,lon=116.3,lat=39.9,**kw):
    return dict(stamp=stamp,longitude=lon,latitude=lat,fix_quality=4,timestamp_source="receiver_utc",
                altitude_ellipsoid=50.,horizontal_std_m=.1,differential_age_s=1.,**kw)


class OutdoorTest(unittest.TestCase):
    def test_unconverted_differential_datum_cannot_set_map_origin(self):
        state=OutdoorState(clock=lambda:100)
        for datum in ('CGCS2000','ITRF2008','unknown'):
            state.feed_fix(fix(100,coordinate_system=datum))
            self.assertIsNone(state.position)
            self.assertIsNone(state.origin)
        state.feed_fix(fix(100,coordinate_system='WGS84'))
        self.assertIsNotNone(state.position)

    def test_geodesy_roundtrip_and_axis(self):
        origin=GeoOrigin(116.3,39.9,50)
        for east,north in ((0,0),(10,0),(0,100),(-300,500)):
            p=origin.wgs84(east,north)
            q=origin.enu(*p,50)
            self.assertAlmostEqual(q[0],east,places=3)
            self.assertAlmostEqual(q[1],north,places=3)

    def test_rigid_fit_and_stationary_rejection(self):
        pairs=[((i,0),(20,30+i)) for i in range(24)]
        result=fit_alignment(pairs)
        self.assertAlmostEqual(result['yaw'],math.pi/2)
        self.assertAlmostEqual(result['tx'],20)
        self.assertLess(result['residual_m'],1e-10)
        self.assertIsNone(fit_alignment([((0,0),(20,30))]*40))
        self.assertIsNone(fit_alignment([((i,0),(2*i,0)) for i in range(24)]))
        pairs[-1]=((23,0),(99,99))
        self.assertIsNone(fit_alignment(pairs))

    def test_saved_origin_not_saved_odom_alignment(self):
        with tempfile.TemporaryDirectory() as tmp:
            state=OutdoorState(tmp,clock=lambda:100)
            state.origin=GeoOrigin(116.3,39.9,50)
            state.set_route(route())
            state.alignment={"yaw":1.,"tx":2.,"ty":3.,"stamp":100}
            state.save_session()
            restored=OutdoorState(tmp)
            self.assertEqual(restored.origin.values,state.origin.values)
            self.assertEqual(restored.route,state.route)
            self.assertIsNone(restored.alignment)
            self.assertIsNone(restored.snapshot()['position'])
            self.assertFalse(restored.guiding)

    def test_changing_route_archives_previous_track_without_relabeling(self):
        with tempfile.TemporaryDirectory() as tmp:
            state=OutdoorState(tmp,clock=lambda:100);state.set_route(route())
            old=state.session_id;state.feed_fix(fix(100))
            state.set_route(dict(route(),title='第二条路线'))
            saved=json.loads((Path(tmp)/f'session-{old}.json').read_text(encoding='utf-8'))
            self.assertEqual(saved['route']['title'],'验证路线')
            self.assertEqual(len(saved['track']),1)
            self.assertFalse(state.track)

    def test_bad_route_coordinate_range_size_and_atomic_preservation(self):
        state=OutdoorState()
        good=state.set_route(route())
        for bad in ({**route(),'coordinate_system':'WGS84'}, {**route(),'points':[[float('nan'),39.9],[116.3,40]]},
                    {**route(),'points':[[181,40],[116.3,40]]}, {**route(),'points':[[116.3,40]]*2001},
                    {**route(),'points':[[116.3,40],[116.3,40]]}):
            with self.assertRaises(ValueError):state.set_route(bad)
            self.assertEqual(state.route,good)

    def test_no_fix_and_stale_never_ready(self):
        now=[100.]
        state=OutdoorState(clock=lambda:now[0]);state.set_route(route())
        for data in (dict(fix(100),fix_quality=0),fix(98),fix(101),dict(fix(100),longitude=None),dict(fix(100),timestamp_source='receive')):
            state.feed_fix(data)
            self.assertIsNone(state.snapshot()['position'])
        with self.assertRaises(ValueError):state.control('start')
        state.feed_fix(fix(100))
        self.assertIsNotNone(state.snapshot()['position'])
        self.assertIsNone(state.alignment)
        self.assertIn('安装偏移',state.alignment_reason)
        now[0]=102
        self.assertIsNone(state.snapshot()['position'])

    def test_out_of_order_jump_and_no_fake_precision(self):
        now=[100.]
        state=OutdoorState(clock=lambda:now[0])
        state.feed_fix(fix(100))
        now[0]=100.1;state.feed_fix(fix(100.1,lon=116.4))
        self.assertEqual(state.position['wgs84'],[116.3,39.9])
        self.assertIn('跳变',state.observation_reason)
        state.feed_fix(fix(100))
        self.assertIn('乱序',state.observation_reason)

    def test_full_alignment_lever_arm_loss_timeout_reset(self):
        now=[100.]
        state=OutdoorState(clock=lambda:now[0],calibration=dict(time_verified=True,lever_arm_verified=True,antenna_in_body_m=[1,0,0]))
        state.origin=GeoOrigin(116.3,39.9,50)
        for i in range(25):
            now[0]=100+i+.02
            for dt in (-.04,.04):
                state.feed_odom(100+i+dt,[i+dt,0,0],[0,0,0,1])
            # 90 degree alignment. Antenna ahead of body by 1 m in body X.
            p=state.origin.wgs84(10,20+i+1)
            state.feed_fix(fix(100+i,lon=p[0],lat=p[1]))
        self.assertIsNotNone(state.alignment)
        self.assertAlmostEqual(state.alignment['yaw'],math.pi/2,places=4)
        self.assertAlmostEqual(state.alignment['ty'],20,places=3)
        self.assertEqual(state.snapshot()['position']['source'],'gnss_lio')
        for step in range(1241,1271):
            now[0]=step/10
            state.feed_odom(now[0],[now[0]-100,0,0],[0,0,0,1])
        self.assertTrue(state.snapshot()['position']['coasting'])
        for step in range(1271,1361):
            now[0]=step/10
            state.feed_odom(now[0],[now[0]-100,0,0],[0,0,0,1])
        self.assertTrue(state.snapshot()['position']['coasting'])
        now[0]=137  # Missing output, rather than satellite age, stops guidance.
        self.assertIsNone(state.snapshot()['position'])
        state.reset_odometry();self.assertIsNone(state.alignment)

    def test_unreported_accuracy_cannot_initialize(self):
        state=OutdoorState(clock=lambda:100,calibration=dict(time_verified=True,lever_arm_verified=True,antenna_in_body_m=[0,0,0]))
        state.feed_fix(dict(fix(100),horizontal_std_m=None))
        self.assertIsNone(state.alignment)
        self.assertIn('精度',state.alignment_reason)

    def covariance_fixture(self):
        now=[100.]
        state=OutdoorState(clock=lambda:now[0],calibration=dict(
            time_verified=True,lever_arm_verified=True,antenna_in_body_m=[0,0,0]))
        state.feed_odom(99.96,[0,0,0],[0,0,0,1])
        state.feed_odom(100.04,[0,0,0],[0,0,0,1])
        original=dict(fix(100),received_at=100.,horizontal_std_m=None,
                      covariance=None,covariance_source='unknown')
        update=dict(original,horizontal_std_m=.1,covariance_source='receiver_gst',
                    covariance=[.01,0.,0.,0.,.01,0.,0.,0.,.04],covariance_update=True)
        state.feed_fix(original)
        return state,now,original,update

    def test_late_covariance_aligns_once_without_new_position_or_track(self):
        state,now,original,update=self.covariance_fixture()
        position=dict(state.position)
        self.assertEqual(len(state.pairs),0)
        now[0]=100.1
        state.feed_fix(update)
        self.assertEqual(len(state.pairs),1)
        self.assertEqual(state.position,position)
        self.assertEqual(state.last_accepted,(100,[116.3,39.9]))
        self.assertEqual(len(state.track),1)
        state.feed_fix(update)
        self.assertEqual(len(state.pairs),1)
        self.assertIn('乱序',state.observation_reason)
        now[0]=102
        self.assertIsNone(state.snapshot()['position'])

    def test_covariance_update_cannot_bypass_epoch_value_or_freshness_checks(self):
        changes=(
            {'stamp':99.9},{'stamp':100.1},{'longitude':116.300001},
            {'fix_quality':5},{'altitude_ellipsoid':51.},
            {'received_at':100.1},{'differential_age_s':0.},
            {'covariance_update':False},{'covariance_source':'unknown'},
            {'horizontal_std_m':None},{'horizontal_std_m':0.},
        )
        for change in changes:
            with self.subTest(change=change):
                state,now,original,update=self.covariance_fixture()
                state.feed_fix(dict(update,**change))
                self.assertEqual(len(state.pairs),0)
                self.assertEqual(state.position_id,1)
                self.assertEqual(len(state.track),1)
        state,now,original,update=self.covariance_fixture()
        now[0]=102
        state.feed_fix(update)
        self.assertEqual(len(state.pairs),0)
        self.assertIn('过期',state.observation_reason)

    def test_late_covariance_preserves_quality_and_precision_gates(self):
        for change in ({'horizontal_std_m':.6},{'fix_quality':5},{'differential_age_s':6.}):
            with self.subTest(change=change):
                state,now,original,update=self.covariance_fixture()
                # Only the precision may change in the update; other gates belong
                # to the original GGA and must remain unchanged.
                if 'horizontal_std_m' not in change:
                    state=OutdoorState(clock=lambda:100,calibration=state.calibration)
                    state.feed_odom(99.96,[0,0,0],[0,0,0,1])
                    state.feed_odom(100.04,[0,0,0],[0,0,0,1])
                    state.feed_fix(dict(original,**change))
                state.feed_fix(dict(update,**change))
                self.assertEqual(len(state.pairs),0)

    def test_rejected_frame_does_not_replace_pending_covariance_identity(self):
        state,now,original,update=self.covariance_fixture()
        state.feed_fix(dict(original,stamp=99.9))
        state.feed_fix(update)
        self.assertEqual(len(state.pairs),1)
        self.assertEqual(state.position_id,1)

    def test_route_guidance_stop_robot_gate_and_segment_projection(self):
        now=[100.];state=OutdoorState(clock=lambda:now[0]);state.set_route(route())
        state.feed_fix(fix(100));state.update_conversion(state.position,[116.3005,39.9])
        state.mode='robot'
        with self.assertRaises(ValueError):state.control('start')
        state.mode='walking';state.control('start')
        guide=state.snapshot()['guidance'];self.assertLess(guide['off_route_m'],.01)
        self.assertGreater(guide['remaining_m'],100)
        with self.assertRaises(ValueError):state.set_route(route())
        now[0]=102;self.assertIsNone(state.snapshot()['guidance'])
        state.control('stop');self.assertFalse(state.guiding)

    def test_closed_loop_does_not_arrive_at_start(self):
        state=OutdoorState(clock=lambda:100)
        state.set_route(dict(route(),points=[[116.3,39.9],[116.301,39.9],[116.301,39.901],[116.3,39.9]]))
        state.feed_fix(fix(100));state.update_conversion(state.position,[116.3,39.9]);state.control('start')
        guide=state.snapshot()['guidance']
        self.assertFalse(guide['near_destination'])
        self.assertGreater(guide['remaining_m'],300)

    def test_simulation_isolated_and_never_resumed_or_saved_as_real(self):
        with tempfile.TemporaryDirectory() as tmp:
            now=[100.];state=OutdoorState(tmp,clock=lambda:now[0]);state.set_route(route())
            state.simulate('enter');state.control('start')
            for _ in range(6):
                now[0]+=2;view=state.snapshot()
            self.assertGreater(view['simulation']['progress_m'],30)
            self.assertIsNone(view['position'])
            self.assertEqual(view['bd09']['source'],'simulation')
            self.assertFalse(state.track);self.assertFalse(state.track_bd09)
            self.assertIsNone(state.position);self.assertIsNone(state.origin)
            with self.assertRaises(ValueError):state.control('save')
            with self.assertRaises(ValueError):state.set_route(route())
            state.save_session()  # Shutdown preserves only real-data fields.
            doc=json.loads((Path(tmp)/'active.json').read_text(encoding='utf-8'))
            self.assertFalse(doc['track']);self.assertFalse(doc['track_bd09'])
            self.assertTrue(doc['simulation_active']);self.assertFalse(doc['simulation_track_included'])
            restored=OutdoorState(tmp)
            self.assertIsNone(restored.snapshot()['simulation'])
            self.assertIsNone(restored.snapshot()['bd09'])
            state.control('stop')
            self.assertIsNone(state.snapshot()['simulation'])
            with self.assertRaises(ValueError):state.control('start')

    def test_simulated_deviation_loss_recovery_arrival_and_robot_mode(self):
        now=[100.];state=OutdoorState(clock=lambda:now[0]);state.set_route(route())
        state.mode='robot'
        with self.assertRaises(ValueError):state.simulate('enter')
        state.mode='walking';state.simulate('enter');state.simulate('walk')
        now[0]+=2;self.assertFalse(state.snapshot()['guidance']['near_destination'])
        state.simulate('offroute')
        self.assertGreater(state.snapshot()['guidance']['off_route_m'],29)
        state.simulate('lost')
        self.assertIsNone(state.snapshot()['bd09']);self.assertIsNone(state.snapshot()['guidance'])
        state.simulate('recover')
        self.assertLess(state.snapshot()['guidance']['off_route_m'],.1)
        for _ in range(35):now[0]+=2;view=state.snapshot()
        self.assertTrue(view['guidance']['near_destination'])
        state.simulate('restart');self.assertEqual(state.snapshot()['simulation']['progress_m'],0)
        state.mode='robot';self.assertIsNone(state.snapshot()['simulation'])

    def test_real_fix_survives_simulation_exit_and_display_suspension_pauses(self):
        now=[100.];state=OutdoorState(clock=lambda:now[0]);state.set_route(route())
        state.simulate('enter');state.simulate('walk')
        now[0]+=20
        view=state.snapshot();self.assertFalse(view['simulation']['running'])
        self.assertEqual(view['simulation']['progress_m'],0)
        state.feed_fix(fix(now[0]));state.update_conversion(state.position,[116.3,39.9])
        state.simulate('exit');self.assertEqual(state.snapshot()['position']['source'],'gnss')
        self.assertEqual(len(state.track),1)


class OutdoorHTTPTest(unittest.TestCase):
    def setUp(self):
        self.state=RemoteState();self.outdoor=OutdoorState()
        self.services=MapServices(url='',token='')
        self.server=RemoteServer(self.state,'127.0.0.1',0,[],outdoor=self.outdoor,map_services=self.services)
        self.token=self.state.pair(self.state.code)
    def tearDown(self):self.server.close()
    def request(self,path,data=None,auth=True,origin=None):
        connection=http.client.HTTPConnection('127.0.0.1',self.server.port,timeout=3)
        headers={}
        if auth:headers['Cookie']='nav_remote='+self.token
        if data is not None:headers['Content-Type']='application/json'
        if origin:headers['Origin']=origin
        connection.request('POST' if data is not None else 'GET',path,body=json.dumps(data) if data is not None else None,headers=headers)
        response=connection.getresponse();result=response.status,response.read(),dict(response.getheaders());connection.close();return result
    def test_location_and_export_require_pairing(self):
        for path in ('/outdoor/config','/outdoor/state','/outdoor/export'):
            self.assertEqual(self.request(path,auth=False)[0],401)
        self.assertEqual(self.request('/outdoor/route',route(),auth=False)[0],401)
    def test_route_saved_shared_and_csrf_rejected(self):
        self.assertEqual(self.request('/outdoor/route',route())[0],200)
        self.assertEqual(json.loads(self.request('/outdoor/state')[1])['route'],self.outdoor.route)
        self.assertEqual(self.request('/outdoor/control',{'action':'stop'},origin='https://evil.test')[0],403)
        self.assertEqual(self.request('/outdoor/route',{**route(),'coordinate_system':'WGS84'})[0],400)
    def test_page_policy_does_not_relax_remote_screen(self):
        self.assertIn('baidu.com',self.request('/outdoor')[2]['Content-Security-Policy'])
        self.assertNotIn('baidu.com',self.request('/')[2]['Content-Security-Policy'])
        self.assertNotIn('unsafe-eval',self.request('/')[2]['Content-Security-Policy'])
        self.assertEqual(self.request('/outdoor')[2]['Referrer-Policy'],'strict-origin')
    def test_revoke_hides_route(self):
        self.state.revoke()
        self.assertEqual(self.request('/outdoor/state')[0],401)

    def test_unified_url_has_same_state_pairing_and_map_policy(self):
        self.assertEqual(self.request('/navigation/state',auth=False)[0],401)
        self.assertEqual(self.request('/navigation/map',{'action':'anchor'},auth=False)[0],401)
        self.assertEqual(self.request('/navigation/map',{'action':'anchor'},origin='https://evil.test')[0],403)
        self.assertEqual(self.request('/navigation/route',route())[0],200)
        status,body,headers=self.request('/navigation')
        self.assertEqual(status,200)
        self.assertIn('导航与建图',body.decode())
        self.assertIn('baidu.com',headers['Content-Security-Policy'])
        state=json.loads(self.request('/navigation/state')[1])
        self.assertEqual(state['route'],self.outdoor.route)
        self.assertIn('map_navigation_ready',state['navigation'])

    def test_geographic_map_control_requires_pairing_and_never_accepts_paths(self):
        self.assertEqual(self.request('/outdoor/map',{'action':'start'},auth=False)[0],401)
        self.assertEqual(self.request('/outdoor/map',{'action':'start'},origin='https://evil.test')[0],403)
        self.assertEqual(self.request('/outdoor/map',{'action':'select','id':'../../secret'})[0],400)
        self.assertEqual(self.request('/outdoor/map',{'action':'select','id':{'bad':1}})[0],400)
        self.assertEqual(self.request('/outdoor/map',{'action':'start'})[0],400)
        self.assertFalse(self.outdoor.requests)

    def test_simulation_requires_pairing_and_same_origin(self):
        self.outdoor.set_route(route())
        self.assertEqual(self.request('/outdoor/simulation',{'action':'enter'},auth=False)[0],401)
        self.assertEqual(self.request('/outdoor/simulation',{'action':'enter'},origin='https://evil.test')[0],403)
        self.assertEqual(self.request('/outdoor/simulation',{'action':'enter'})[0],200)
        view=json.loads(self.request('/outdoor/state')[1])
        self.assertTrue(view['simulation']['active'])
        self.assertIsNone(view['position'])
        self.assertEqual(self.request('/outdoor/control',{'action':'save'})[0],400)


if __name__=='__main__':unittest.main()
