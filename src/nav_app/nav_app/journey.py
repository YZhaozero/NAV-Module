"""Bounded walking route graph across metric maps and geographic roads.

Explicit portals relate *places*, never estimate a geodetic transform. Metric
edges are planned on verified occupancy grids, with unknown cells blocked.
No TF, ROS action or robot command is emitted by this module.
"""
import copy
import hashlib
import heapq
import json
import math
import time
from pathlib import Path
import uuid

import numpy as np

from .geo_maps import atomic_json, file_hash, vector


def separation(a, b, domain):
    if domain == 'BD09':
        from .outdoor import distance
        return distance(a, b)
    return math.dist(a[:2], b[:2])


def grid_fingerprint(path):
    import yaml
    path = Path(path).resolve(strict=True)
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    image = (path.parent / config['image']).resolve(strict=True)
    return hashlib.sha256((file_hash(path)+file_hash(image)).encode()).hexdigest()


class PlanningBudgetExceeded(ValueError):
    pass


def grid_path(grid, start, goal, clearance=.25, max_cells=500_000, deadline=None):
    """A* with a circular footprint, blocked boundary and no diagonal cutting."""
    occ = grid.occupancy
    if (occ.size > max_cells or not np.isfinite(grid.origin).all()
            or not math.isfinite(grid.resolution) or grid.resolution <= 0
            or not vector(start, 2) or not vector(goal, 2)):
        raise ValueError('通行地图过大或坐标无效')
    h, w = occ.shape
    blocked = occ != 0
    radius = max(1, math.ceil(clearance/grid.resolution))
    if radius > 15:
        raise ValueError('通行地图分辨率过细')
    padded = np.pad(blocked, radius, constant_values=True)
    inflated = blocked.copy()
    for dy in range(-radius, radius+1):
        for dx in range(-radius, radius+1):
            # Conservative rasterized footprint, including cell extents.
            if dx*dx+dy*dy <= (radius+.71)**2:
                inflated |= padded[radius+dy:radius+dy+h, radius+dx:radius+dx+w]
    co, si = math.cos(grid.origin[2]), math.sin(grid.origin[2])
    def cell(p):
        x, y = p[0]-grid.origin[0], p[1]-grid.origin[1]
        return (math.floor((co*x+si*y)/grid.resolution),
                math.floor((-si*x+co*y)/grid.resolution))
    def free(p):
        return 0 <= p[0] < w and 0 <= p[1] < h and not inflated[p[1], p[0]]
    a, b = cell(start), cell(goal)
    if not free(a) or not free(b):
        raise ValueError('起点或终点靠近障碍、未知区域或地图边缘')
    costs, parents = {a: 0.}, {}
    queue = [(math.dist(a, b), 0., a)]
    expanded=0
    while queue:
        expanded+=1
        if expanded%1024==0 and (expanded>200_000 or (deadline is not None and time.monotonic()>deadline)):
            raise PlanningBudgetExceeded('路线计算范围过大，请减少途经地点或缩小通行地图')
        _, g, p = heapq.heappop(queue)
        if g != costs[p]:
            continue
        if p == b:
            chain = [b]
            while chain[-1] != a:
                chain.append(parents[chain[-1]])
            points = []
            for x, y in reversed(chain):
                x, y = (x+.5)*grid.resolution, (y+.5)*grid.resolution
                points.append([co*x-si*y+float(grid.origin[0]), si*x+co*y+float(grid.origin[1])])
            # Exact endpoints remain inside footprint-checked cells.
            return [list(start), *points, list(goal)]
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            q = p[0]+dx, p[1]+dy
            if not free(q) or (dx and dy and (not free((p[0]+dx,p[1])) or not free((p[0],p[1]+dy)))):
                continue
            cost = g+math.hypot(dx,dy)
            if cost < costs.get(q, math.inf):
                costs[q], parents[q] = cost, p
                heapq.heappush(queue, (cost+math.dist(q,b), cost, q))
    raise ValueError('通行地图中没有连接这两个地点的路线')


class JourneyNetwork:
    def __init__(self, directory, maps):
        self.path = Path(directory)/'network.json' if directory else None
        self.maps = maps
        self.data = dict(schema=1, places=[], links=[])
        self.error = ''
        if self.path and self.path.exists():
            try:
                if self.path.stat().st_size > 256_000:
                    raise ValueError('地点网络文件过大')
                data = json.loads(self.path.read_text(encoding='utf-8'))
                self.validate(data)
                self.data = data
            except (OSError, ValueError, KeyError, TypeError):
                self.error = '地点网络无法读取，请重新导入；原文件仍保留'

    def validate(self, data):
        from .outdoor import coordinate, OutdoorState
        if (not isinstance(data,dict) or data.get('schema') != 1
                or not isinstance(data.get('places'),list) or len(data['places']) > 32
                or not isinstance(data.get('links'),list) or len(data['links']) > 64):
            raise ValueError('地点网络格式无效，最多32个地点和64条连接')
        nodes = {}
        for p in data['places']:
            if (not isinstance(p,dict) or not isinstance(p.get('id'),str)
                    or not 1 <= len(p['id']) <= 64 or p['id'] in nodes
                    or not isinstance(p.get('name'),str) or not 1 <= len(p['name'].strip()) <= 80
                    or not vector(p.get('point'),2)):
                raise ValueError('地点名称、编号或坐标无效')
            if p.get('domain') == 'BD09':
                if not coordinate(p['point']):
                    raise ValueError('路网地点坐标无效')
            else:
                m = self.maps.get(p.get('domain'))
                if p.get('map_sha256') != m['sha256']:
                    raise ValueError('地点与地图版本不一致，请重新登记')
            nodes[p['id']] = p
        seen = set()
        for link in data['links']:
            if not isinstance(link,dict) or not isinstance(link.get('from'),str) or not isinstance(link.get('to'),str):
                raise ValueError('连接端点无效')
            a,b = nodes.get(link['from']),nodes.get(link['to'])
            if not a or not b or a is b or (a['id'],b['id']) in seen or type(link.get('bidirectional')) is not bool:
                raise ValueError('连接端点缺失、重复或方向无效')
            seen.add((a['id'],b['id']))
            if link.get('kind') == 'portal':
                if (a['domain'] == b['domain'] or link.get('passage_confirmed') is not True
                        or not isinstance(link.get('name'),str) or not 1 <= len(link['name']) <= 80):
                    raise ValueError('地图入口须明确确认可通行，并连接两个不同坐标域')
            elif link.get('kind') == 'road' and a['domain'] == b['domain'] == 'BD09':
                route = OutdoorState.validate_route(link.get('route'))
                if (separation(route['points'][0],a['point'],'BD09') > 5
                        or separation(route['points'][-1],b['point'],'BD09') > 5):
                    raise ValueError('道路路线没有接上所选地点')
            else:
                raise ValueError('连接类型无效；地图内路线由通行地图计算')
        return nodes

    def replace(self, data):
        self.validate(data)
        # Strip no data silently: reject non-JSON/NaN and oversized input.
        encoded = json.dumps(data, ensure_ascii=False, allow_nan=False)
        if len(encoded.encode()) > 256_000:
            raise ValueError('地点网络过大')
        if self.path:
            atomic_json(self.path,data)
        self.data, self.error = copy.deepcopy(data), ''

    def plan(self, start, end):
        from .scene_catalog import read_scene
        from .map_processing import load_grid_map
        nodes = self.validate(self.data)
        if start not in nodes or end not in nodes or start == end:
            raise ValueError('请选择两个不同的已登记地点')
        graph = {k:[] for k in nodes}
        bindings = {}
        deadline=time.monotonic()+6.
        def edge(a,b,kind,points,**extra):
            cost = sum(separation(x,y,nodes[a]['domain']) for x,y in zip(points,points[1:])) if kind != 'portal' else 1.
            graph[a].append(dict(start=a,end=b,kind=kind,domain=nodes[a]['domain'],points=points,
                                 target_domain=nodes[b]['domain'],target_point=nodes[b]['point'],
                                 length_m=cost,**extra))
        for link in self.data['links']:
            a,b = link['from'],link['to']
            points = link['route']['points'] if link['kind']=='road' else [nodes[a]['point']]
            edge(a,b,link['kind'],points,name=link.get('name','道路路线'))
            if link['bidirectional']:
                edge(b,a,link['kind'],list(reversed(points)) if link['kind']=='road' else [nodes[b]['point']],name=link.get('name','道路路线'))
        for domain in sorted({p['domain'] for p in nodes.values()}-{'BD09'}):
            m = self.maps.get(domain,verify=True)
            if not m.get('scene_path'):
                continue
            scene = read_scene(m['scene_path'])
            if not scene.ready or Path(scene.cloud).resolve() != Path(m['cloud_path']).resolve():
                raise ValueError('场景的通行地图与定位地图尚未核对')
            fingerprint = grid_fingerprint(scene.grid)
            if fingerprint != m.get('grid_sha256'):
                raise ValueError('通行地图已变化，请在设备上重新载入地图包')
            bindings[domain] = dict(map_sha256=m['sha256'],grid_sha256=fingerprint)
            grid = load_grid_map(scene.grid)
            group = [p for p in nodes.values() if p['domain']==domain]
            for i,a in enumerate(group):
                for b in group[i+1:]:
                    try:
                        if time.monotonic()>deadline:raise PlanningBudgetExceeded('地点较多，请缩小本次规划范围')
                        points = grid_path(grid,a['point'],b['point'],deadline=deadline)
                    except PlanningBudgetExceeded:
                        raise
                    except ValueError:
                        continue
                    edge(a['id'],b['id'],'local',points,name=scene.title)
                    edge(b['id'],a['id'],'local',list(reversed(points)),name=scene.title)
        queue, costs, previous = [(0.,start)],{start:0.},{}
        while queue:
            cost, node = heapq.heappop(queue)
            if cost != costs[node]:
                continue
            if node == end:
                segments=[]
                while node != start:
                    segment=previous[node];segments.append(segment);node=segment['start']
                return dict(schema=1,id=uuid.uuid4().hex,title=nodes[end]['name'],
                            start=start,end=end,segments=list(reversed(segments)),bindings=bindings,
                            distance_m=sum(s['length_m'] for s in segments if s['kind']!='portal'),
                            network_sha256=hashlib.sha256(json.dumps(self.data,sort_keys=True).encode()).hexdigest())
            for segment in graph[node]:
                target=segment['end'];new=cost+segment['length_m']
                if new < costs.get(target,math.inf):
                    costs[target],previous[target]=new,segment
                    heapq.heappush(queue,(new,target))
        raise ValueError('地点之间尚无已确认的连接，或通行地图阻断了路线')


class JourneySession:
    def __init__(self, directory=None):
        self.path=Path(directory)/'journey.json' if directory else None
        self.plan=None;self.index=0;self.phase='idle';self.reason='选择地点规划全程路线'
        self.progress=0.;self.remaining=0.;self._near_since=None;self._last=0.
        if self.path and self.path.exists():
            try:
                if self.path.stat().st_size > 4_000_000:raise ValueError()
                old=json.loads(self.path.read_text(encoding='utf-8'))
                # A restart never resumes motion, localization or a partial route.
                if isinstance(old.get('plan'),dict):self.reason='上次路线已停止，请重新规划后开始'
            except (OSError,ValueError):self.reason='上次路线记录无法读取，请重新规划'

    @property
    def active(self):
        return self.phase in ('guiding','lost','transition','deviated')

    @property
    def segment(self):
        return self.plan['segments'][self.index] if self.plan and self.index<len(self.plan['segments']) else None

    def save(self):
        if self.path:atomic_json(self.path,self.snapshot())

    def preview(self,plan):
        if self.active:raise ValueError('请先结束全程引导')
        self.plan=copy.deepcopy(plan);self.index=0;self.progress=0.;self.phase='preview'
        self.reason=f"共{len(plan['segments'])}段，约{plan['distance_m']:.0f}米，请到起点后开始"
        self.save()

    def stop(self):
        self.phase='stopped';self.reason='全程引导已结束，地点已保留';self._near_since=None
        self.save()

    def point(self, domain, navigation, converted, now):
        if navigation.get('simulation'):return None
        if domain=='BD09':
            p=navigation.get('geographic_pose')
            if p and converted and 0<=now-converted.get('stamp',0)<=4:return converted['point']
        else:
            p=navigation.get('local_pose')
            binding=(self.plan or {}).get('bindings',{}).get(domain)
            if (p and binding and p.get('map_id')==domain and p.get('map_sha256')==binding['map_sha256']
                    and -.05<=now-p.get('stamp',0)<=.5):return p['position_m'][:2]
        return None

    def begin(self, navigation, converted, now):
        if self.phase!='preview' or not self.segment:raise ValueError('请先规划全程路线')
        s=self.segment;p=self.point(s['domain'],navigation,converted,now)
        if p is None or separation(p,s['points'][0],s['domain']) > (5 if s['domain']=='BD09' else 1):
            raise ValueError('当前位置尚未确认在路线起点附近')
        self.phase='guiding';self._last=now;self.save()

    def update(self,navigation,converted,now):
        if not self.active or not self.segment:return
        s=self.segment;domain=s['domain']
        if s['kind']=='portal':
            self.phase='transition';self.reason=f"{s['name']}：请在入口切换定位，确认下一段位置"
            return
        p=self.point(domain,navigation,converted,now)
        if p is None:
            self.phase='lost';self.reason='定位暂不可用，全程引导暂停';self._near_since=None;self._last=now;return
        points=s['points'];radius=5 if domain=='BD09' else .8
        scale=111195*math.cos(math.radians(p[1])) if domain=='BD09' else 1
        sy=111195 if domain=='BD09' else 1
        prefix=0.;best=None
        advance=min(8.,max(2.,(now-self._last)*2+1));self._last=now
        for a,b in zip(points,points[1:]):
            ax,ay=(a[0]-p[0])*scale,(a[1]-p[1])*sy
            dx,dy=(b[0]-a[0])*scale,(b[1]-a[1])*sy
            length=math.hypot(dx,dy);t=max(0.,min(1.,-(ax*dx+ay*dy)/max(length**2,1e-12)))
            along=prefix+t*length;gap=math.hypot(ax+t*dx,ay+t*dy)
            if self.progress-1<=along<=self.progress+advance and (best is None or gap<best[0]):best=gap,along,b
            prefix+=length
        if best is None or best[0]>(12 if domain=='BD09' else 1.5):
            self.phase='deviated';self.reason='已偏离路线，请停下确认或重新规划';self._near_since=None;return
        self.progress=max(self.progress,best[1]);self.remaining=max(0.,prefix-self.progress)
        self.phase='guiding';self.reason=f"第{self.index+1}/{len(self.plan['segments'])}段 · 剩余约{self.remaining:.0f}米"
        heading=(navigation.get('geographic_pose') or {}).get('heading_enu_rad') if domain=='BD09' else (navigation.get('local_pose') or {}).get('heading_rad')
        if heading is not None:
            prefix=0.;target=points[-1]
            for a,b in zip(points,points[1:]):
                length=separation(a,b,domain)
                if prefix+length>=self.progress+1.5:
                    t=(self.progress+1.5-prefix)/max(length,1e-9)
                    target=[a[i]+t*(b[i]-a[i]) for i in (0,1)];break
                prefix+=length
            angle=math.atan2((target[1]-p[1])*sy,(target[0]-p[0])*scale)-heading
            angle=math.atan2(math.sin(angle),math.cos(angle))
            instruction='请转身' if abs(angle)>2.2 else '向左转' if angle>.6 else '向右转' if angle<-.6 else '继续向前'
            self.reason=instruction+' · '+self.reason
        near=self.remaining<radius and separation(p,points[-1],domain)<radius
        self._near_since=(now if self._near_since is None else self._near_since) if near else None
        if self._near_since is not None and now-self._near_since>=1:
            self._advance()

    def _advance(self):
        self.index+=1;self.progress=0.;self._near_since=None
        if not self.segment:self.phase='arrived';self.reason='已到达全程目的地'
        elif self.segment['kind']=='portal':self.phase='transition';self.reason='已到入口，请确认下一段定位'
        self.save()

    def confirm_transition(self,navigation,converted,now):
        s=self.segment
        if self.phase!='transition' or not s or s['kind']!='portal':raise ValueError('当前没有待确认的地图入口')
        p=self.point(s['target_domain'],navigation,converted,now)
        if p is None or separation(p,s['target_point'],s['target_domain'])>(5 if s['target_domain']=='BD09' else 1):
            raise ValueError('下一段入口附近的定位尚未确认，请先启动对应地图定位')
        self.phase='guiding';self._last=now;self._advance()

    def snapshot(self):
        return dict(plan=copy.deepcopy(self.plan),index=self.index,phase=self.phase,reason=self.reason,
                    progress_m=self.progress,remaining_m=self.remaining,segment=copy.deepcopy(self.segment))


class JourneyCommands:
    """Shared Qt/paired-phone commands. Caller owns the OutdoorState RLock."""
    def init_journey(self):
        self.network=JourneyNetwork(self.directory,self.maps)
        self.journey=JourneySession(self.directory)

    def journey_command(self,data):
        action=data.get('action')
        if action=='start':
            # File hashing runs on the HTTP/GUI worker, never the Qt timer.
            from .scene_catalog import read_scene
            with self.lock:plan=copy.deepcopy(self.journey.plan)
            for identifier,binding in (plan or {}).get('bindings',{}).items():
                m=self.maps.get(identifier,verify=True)
                if m['sha256']!=binding['map_sha256'] or grid_fingerprint(read_scene(m['scene_path']).grid)!=binding['grid_sha256']:
                    raise ValueError('地图已变化，请重新载入并规划路线')
        if action=='plan':
            with self.lock:
                if self.journey.active or self.guiding or self.local_navigation_active or self.simulation or self.sensor_replay:
                    raise ValueError('请先结束当前任务并退出回放或模拟')
                saved=copy.deepcopy(self.network.data)
            network=JourneyNetwork(None,self.maps);network.data=saved
            plan=network.plan(data.get('start'),data.get('end'))
            with self.lock:
                if (self.network.data!=saved or self.journey.active or self.guiding or self.local_navigation_active
                        or self.simulation or self.sensor_replay):
                    raise ValueError('地点或任务状态已变化，请重新规划')
                self.journey.preview(plan)
            return self.journey.snapshot()
        with self.lock:
            if action=='stop':
                self.journey.stop()
                self.requests=type(self.requests)(r for r in self.requests if not isinstance(r,dict) or r.get('action')!='journey_switch')
                return self.journey.snapshot()
            if self.mode!='walking' or self.simulation or self.sensor_replay:
                raise ValueError('全程路线用于步行引导，请先退出机器人、模拟或回放')
            if self.guiding or self.local_navigation_active:
                raise ValueError('请先结束当前引导任务')
            if action=='start':
                if self.journey.plan is None:raise ValueError('请先规划全程路线')
                if self.journey.plan!=plan:raise ValueError('路线已更新，请重新确认后开始')
                digest=hashlib.sha256(json.dumps(self.network.data,sort_keys=True).encode()).hexdigest()
                if digest!=self.journey.plan['network_sha256']:raise ValueError('地点网络已更新，请重新规划')
                self.journey.begin(self.navigation_snapshot(),self.converted,self.clock())
            elif action=='confirm':
                self.journey.confirm_transition(self.navigation_snapshot(),self.converted,self.clock())
            elif action=='switch':
                s=self.journey.segment
                if self.journey.phase!='transition' or not s or s['kind']!='portal':raise ValueError('尚未到达地图入口')
                if s['target_domain']=='BD09':raise ValueError('下一段使用卫星或已关联地图定位，无需切换地图')
                if self.requests:raise ValueError('设备正在处理上一条请求')
                self.requests.append(dict(action='journey_switch',id=s['target_domain']))
            else:
                if self.journey.active:raise ValueError('请先结束全程引导再编辑地点')
                network=copy.deepcopy(self.network.data)
                if action=='import':network=data.get('network')
                elif action=='place':
                    name=data.get('name','')
                    if data.get('domain')=='BD09':
                        point=data.get('point')
                        place=dict(id=uuid.uuid4().hex,name=name,domain='BD09',point=point)
                    else:
                        p=self.local_map_pose()
                        if not p:raise ValueError('当前位置尚未通过地图匹配，无法登记地点')
                        place=dict(id=uuid.uuid4().hex,name=name,domain=p['map_id'],point=p['position_m'][:2],map_sha256=p['map_sha256'])
                    network['places'].append(place)
                elif action=='link':
                    link=dict(from_=data.get('from'),to=data.get('to'),kind=data.get('kind'),bidirectional=True)
                    link['from']=link.pop('from_')
                    if link['kind']=='portal':link.update(name=data.get('name','入口'),passage_confirmed=data.get('passage_confirmed'))
                    else:link['route']=copy.deepcopy(self.route)
                    network['links'].append(link)
                else:raise ValueError('未知的全程路线操作')
                self.network.replace(network)
            return self.journey.snapshot()

    def journey_snapshot(self):
        with self.lock:
            if self.journey.active:
                if self.mode!='walking' or self.simulation or self.sensor_replay:
                    self.journey.stop()
                else:
                    try:self.journey.update(self.navigation_snapshot(),self.converted,self.clock())
                    except OSError:
                        self.journey.phase='error';self.journey.reason='路线进度保存失败，请检查剩余空间后重新规划'
            return dict(**self.journey.snapshot(),network=copy.deepcopy(self.network.data),error=self.network.error)
