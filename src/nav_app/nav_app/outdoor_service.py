"""Device-side Baidu access, with optional existing gateway compatibility."""
import json
import os
import threading
import time
from urllib.parse import urlsplit
from urllib.request import Request, build_opener, HTTPRedirectHandler
from .outdoor import coordinate
from .baidu_client import BaiduServiceError
from .map_environment import load_map_environment


class ServiceError(ValueError):
    """Only local copy selected by a bounded service code may reach the UI."""


SERVICE_ERRORS={
    'upload_disabled':'服务器尚未开放新数据上传，请联系维护人员配置。',
    'upload_capacity':'服务器上传空间不足，请联系维护人员整理。',
    'upload_integrity':'上传校验失败，请恢复上传；已完成的其他文件会保留。',
    'upload_incomplete':'数据尚未传完，请恢复上传。',
    'upload_missing':'上传记录未找到，请重新选择数据上传。',
}


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class MapServices:
    def __init__(self, url=None, token=None):
        settings = load_map_environment()
        value = lambda name, default="": settings.get(name, os.environ.get(name, default)).strip()
        self.url = (url if url is not None else value("NAV_MAP_GATEWAY")).strip()
        self.token = (token if token is not None else value("NAV_MAP_TOKEN")).strip()
        self.browser_key = value("NAV_BAIDU_BROWSER_AK")
        self.server_key = value("NAV_BAIDU_SERVER_AK")
        self.signing_key = value("NAV_BAIDU_SERVER_SK")
        self.map_mode = value('NAV_MAP_MODE', 'gateway' if self.url else 'direct')

    def call(self, action, data):
        if action not in ('convert','walking'):
            raise ValueError('不支持的服务请求')
        if self.map_mode == 'direct':
            from .baidu_client import call
            if self.signing_key:
                return call(action, data, self.server_key, signing_key=self.signing_key)
            return call(action, data, self.server_key)
        if self.map_mode != 'gateway':raise ValueError('地图连接方式配置无效')
        service_name='地图服务'
        parsed = urlsplit(self.url)
        if not self.token or parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.query or parsed.fragment:
            raise ValueError(service_name+"尚未配置")
        request = Request(self.url.rstrip("/")+"/"+action,
            data=json.dumps(data, allow_nan=False).encode(),
            headers={"Content-Type":"application/json", "Authorization":"Bearer "+self.token}, method="POST")
        try:
            maximum=500000
            with build_opener(NoRedirect()).open(request, timeout=7) as response:
                raw = response.read(maximum+1)
            if len(raw)>maximum:
                raise ValueError()
            result=json.loads(raw)
            if isinstance(result,dict) and not result.get('ok') and result.get('code') in SERVICE_ERRORS:
                raise ServiceError(SERVICE_ERRORS[result['code']])
            if not isinstance(result,dict) or not result.get("ok"):
                raise ValueError()
            return result
        except ServiceError:
            raise
        except Exception:
            # URLs and third-party messages may contain secrets. Never surface them.
            raise ValueError(service_name+"暂时不可用，请检查联网后重试") from None

    def plan(self, start, end, title="室外路线"):
        from .outdoor import OutdoorState
        if not coordinate(start) or not coordinate(end):
            raise ValueError("请选择起点和终点")
        result=self.call("walking", dict(start=start,end=end))
        if len(result.get("points",[])) < 2:
            raise ValueError("这两个点没有可用的步行路线，请调整选点或手动画线")
        return OutdoorState.validate_route(dict(points=result["points"],waypoints=[start,end],
            source="baidu_walking",coordinate_system="BD09",title=title))

    def plan_waypoints(self, waypoints, title='多点步行路线', manual=False):
        from .outdoor import OutdoorState, distance
        if not isinstance(waypoints,list) or not 2 <= len(waypoints) <= 20 or not all(coordinate(p) for p in waypoints):
            raise ValueError('请按顺序选择2～20个途经点')
        if any(distance(a,b)<1 for a,b in zip(waypoints,waypoints[1:])):
            raise ValueError('相邻途经点过近，请撤销重复的点')
        if manual:
            return OutdoorState.validate_route(dict(points=waypoints,waypoints=waypoints,source='manual',coordinate_system='BD09',title='手绘步行路线'))
        points=[]
        for i,(a,b) in enumerate(zip(waypoints,waypoints[1:])):
            try:segment=self.plan(a,b)['points']
            except ValueError as exc:raise ValueError(f'第{i+1}段规划失败：{exc}') from None
            if points and distance(points[-1],segment[0])>15:
                raise ValueError(f'第{i+1}段路线未衔接，请调整该途经点')
            points.extend(segment[1:] if points and points[-1]==segment[0] else segment)
        # Saving happens only once all legs have succeeded.
        return OutdoorState.validate_route(dict(points=points,waypoints=waypoints,source='baidu_walking',coordinate_system='BD09',title=title))


class PositionConverter:
    def __init__(self, state, services):
        self.state,self.services=state,services
        self.stop=threading.Event()
        self.thread=threading.Thread(target=self.run,name="outdoor-map-conversion",daemon=True)
        self.thread.start()

    def run(self):
        last_id=None
        delay=1.
        while not self.stop.wait(delay):
            with self.state.lock:
                position=self.state._position_now()
                display_only=position is None
                if display_only:
                    # A rejected alignment quality is still useful to browse
                    # near the device. Keep it out of the navigation position.
                    reference=self.state.position
                    if (reference and reference.get('source')=='gnss'
                            and reference.get('quality') in (1,2,4,5)
                            and self.state.clock()-reference['stamp']>=-.05):
                        position=dict(reference)
                epoch=self.state.pose_epoch
            identity=(position['id'],display_only,epoch) if position else None
            if not position or identity==last_id:
                continue
            try:
                point=self.convert(position)
                if position.get('source')=='gnss':
                    with self.state.lock:
                        if epoch==self.state.pose_epoch and coordinate(point):
                            self.state._map_display_reference=dict(point=list(point),stamp=position['stamp'],
                                quality=position.get('quality'),source='gnss',epoch=epoch)
                if not display_only:self.state.update_conversion(position,point)
                last_id=identity
                delay=1. if display_only else .2 if self.state.trial.active else 2.
            except BaiduServiceError as exc:
                with self.state.lock:
                    self.state.conversion_error=str(exc)+'；路线引导已暂停'
                delay=10.
            except (ValueError,KeyError,IndexError):
                with self.state.lock:
                    self.state.conversion_error="地图定位转换暂不可用，路线引导已暂停"
                delay=10.

    def convert(self,position):
        from .navigation_trial import LocalMapProjection
        with self.state.lock:
            trial=self.state.trial
            origin=self.state.origin
            active=trial.active and origin is not None
            projection=trial.projection
        if active:
            if projection is None or projection.origin != tuple(origin.values):
                result=self.services.call('convert',{'points':LocalMapProjection.samples(origin)})
                projection=LocalMapProjection(origin.values,result['points'])
                with self.state.lock:
                    if not trial.active or self.state.origin is not origin:
                        raise ValueError('初始对准状态已更新')
                    trial.projection=projection
            east,north,_=origin.enu(*position['wgs84'],origin.values[2])
            return projection.to_bd09([east,north])
        return self.services.call('convert',{'points':[position['wgs84']]})['points'][0]

    def close(self):
        self.stop.set()
        self.thread.join(timeout=.2)
