"""Display-only GNSS on the same Baidu web SDK used by the phone."""
import time
import math
from datetime import datetime, timezone
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from .background_task import BackgroundTask
from .web_map import WebMap
from ..features.trajectory_map import live_position, geographic_track, map_fix, QUALITY, convert_segments


class TrajectoryMap(QWidget):
    def __init__(self, operations):
        super().__init__(); self.operations=operations; self.node=operations.window.node
        self.worker=BackgroundTask(self); self.track_worker=BackgroundTask(self)
        self.zoom=18; self.next_request=0.; self.next_track=0.
        self.context=None; self.generation=0; self.stamp_key=None; self.stamp_changed=0.
        self.point=None; self.point_until=0.; self.point_time=''; self.segments=[]; self.track_signature=None
        self.last_message='正在转换地图坐标…'
        body=QVBoxLayout(self); body.setContentsMargins(0,0,0,0); body.setSpacing(3)
        self.position=QLabel('等待有效卫星位置')
        self.caption=QLabel('网页底图需联网；坐标和原始记录不受地图加载影响。')
        for label in (self.position,self.caption):
            label.setWordWrap(True); label.setTextFormat(Qt.PlainText); label.setStyleSheet('font-size: 15px')
        self.canvas=WebMap()
        body.addWidget(self.position); body.addWidget(self.canvas,1); body.addWidget(self.caption)

    def reset_view(self,delta=0):
        self.zoom=max(3,min(20,self.zoom+delta)); self.next_request=0.; self.next_track=0.
        self.canvas.reset_view(self.zoom)

    def current(self,now):
        if getattr(getattr(self.node,'outdoor',None),'sensor_replay',False):
            return None,'录包回放中，不显示为实时位置',None
        adapter=getattr(self.node,'rtk_live',None)
        fix=getattr(adapter,'observation',None)
        point,label=live_position(fix,getattr(adapter,'observation_received_at',None),now)
        if point and fix.get('stamp') is not None:
            key=(fix.get('timestamp_source'),fix.get('stamp'))
            if key!=self.stamp_key:self.stamp_key,self.stamp_changed=key,now
            elif now-self.stamp_changed>2:return None,'卫星测量时间未更新，暂停显示当前位置',fix
        return point,label,fix

    def send(self,history,message=None):
        if not history and time.monotonic()>self.point_until:self.point=None
        if message is None:message='' if self.point else self.last_message
        self.canvas.send(dict(point=self.point,segments=self.segments,history=history,
                              context=self.generation,message=message))
        if self.point:
            self.caption.setText(('历史位置' if history else '地图位置')+' · '+self.point_time+' · 绿线为轨迹概览')

    def refresh(self,data,history=False,fraction=1.,identity=None):
        now=time.monotonic()
        context=(history,identity,fraction if history else 1.)
        if context!=self.context:
            self.context=context; self.generation+=1; self.next_request=0.; self.next_track=0.
            self.point=None; self.segments=[]; self.track_signature=None
            self.last_message='正在转换地图坐标…'
        rows=data.get('gnss',[])
        if history:
            count=max(1,math.ceil(len(rows)*fraction)) if rows else 0
            fix=next((r for r in reversed(rows[:count]) if map_fix(r)),None)
            point=[fix['longitude'],fix['latitude']] if fix else None
            label='历史位置 · '+QUALITY[fix['fix_quality']] if fix else '此处尚无有效历史位置'
        else:point,label,fix=self.current(now)
        self.position.setText(label+(f'\n经度 {point[0]:.7f} · 纬度 {point[1]:.7f}（WGS84）' if point else ''))
        if not point:
            self.generation+=1; self.point=None; self.send(history,label)
            self.caption.setText('没有有效当前位置；原始记录仍可继续。'); return
        service=getattr(self.node,'map_services',None)
        if service is None:
            self.send(history,'地图服务未连接；坐标和记录仍可使用'); return
        phone=getattr(self.operations,'phone',None)
        remote=getattr(phone,'remote',None)
        origins=getattr(remote,'urls',[])
        origin=origins[0] if origins else f"http://127.0.0.1:{getattr(self.node,'phone_remote_port',8765)}"
        if not self.canvas.ensure(origin,service.browser_key):return
        self.send(history)
        token=self.generation
        if not self.worker.busy and now>=self.next_request:
            self.last_message='正在转换地图坐标…'
            self.next_request=now+1.
            try:observed=datetime.fromtimestamp(fix.get('stamp'),timezone.utc).strftime('%H:%M:%S UTC')
            except (TypeError,ValueError,OverflowError,OSError):observed='时间未知'
            def done(value,error):
                if token!=self.generation:return
                if not history and (time.monotonic()-now>2.5 or self.current(time.monotonic())[0] is None):
                    self.point=None; self.send(history,'等待新的地图位置'); return
                if error:
                    self.point=None; self.next_request=time.monotonic()+10.
                    self.last_message=error; self.send(history,error); return
                self.point=value[0][0]; self.point_time=observed
                self.point_until=now+2.5
                if history:self.next_request=float('inf')
                self.send(history)
            self.worker.start(lambda:convert_segments(service,[[point]]),done)
        # Track conversion cannot delay the current dot.
        if not self.track_worker.busy and now>=self.next_track:
            self.next_track=now+5.
            signature=self.track_signature
            def work():
                segments,_=geographic_track(rows,fraction if history else 1.)
                if segments==signature:return None
                return segments,convert_segments(service,segments)
            def track_done(value,error):
                if token!=self.generation:return
                if not history and self.current(time.monotonic())[0] is None:
                    self.point=None; self.send(history,'位置更新已中断'); return
                if error:
                    self.next_track=time.monotonic()+15.
                    self.caption.setText('轨迹上图暂不可用；原始记录仍完整保留。'); return
                if value is not None:self.track_signature,self.segments=value
                if history:self.next_track=float('inf')
                self.send(history)
            self.track_worker.start(work,track_done)
