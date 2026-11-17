"""One explicit robot route start permits one guard arm attempt; faults latch."""
import json
import time


class MotionSession:
    def __init__(self,client,request_factory,clock=time.monotonic):
        self.client=client;self.request_factory=request_factory;self.clock=clock
        self.received=None;self.state={};self.generation=None;self.started=None;self.future=None;self.armed=False

    def update(self,text,ros_now_ns=None):
        try:
            if len(text)>2048:raise ValueError()
            row=json.loads(text)
            if not isinstance(row,dict):raise ValueError()
            if row.get('schema')!=1 or type(row.get('armed'))!=bool or type(row.get('base_connected'))!=bool:raise ValueError()
            if ros_now_ns is not None and (type(row.get('stamp_ns'))!=int or not -.05<=(ros_now_ns-row['stamp_ns'])/1e9<=.3):raise ValueError()
            self.state=row;self.received=self.clock()
        except (TypeError,ValueError):self.state={};self.received=None

    def fresh(self):return self.received is not None and 0<=self.clock()-self.received<=.3

    def available(self):
        return self.client.service_is_ready() and self.fresh() and self.state.get('base_connected') is True

    def poll(self,generation,running):
        if not running:
            self.started=None;self.future=None;self.armed=False;return ''
        now=self.clock()
        if generation!=self.generation:
            self.generation=generation;self.started=now;self.future=None;self.armed=False
        if self.armed:
            return '' if self.fresh() and self.state.get('armed') is True else '底盘连接或定位异常，已请求停止'
        if self.started is None:return '请重新确认机器人任务'
        if self.future is None and now-self.started>=.2:
            if not self.available():return '底盘未就绪，任务已停止'
            try:self.future=self.client.call_async(self.request_factory())
            except Exception:return '底盘启动失败，任务已停止'
        if self.future is not None and self.future.done():
            try:
                if not self.future.result().success:return '底盘未允许运动，任务已停止'
            except Exception:return '底盘启动未确认，任务已停止'
            if self.fresh() and self.state.get('armed') is True:self.armed=True;return ''
        if now-self.started>3:return '底盘启动超时，任务已停止'
        return ''

    def stop_text(self):
        return '底盘反馈已静止' if self.fresh() and self.state.get('stop_confirmed') is True else '等待底盘静止反馈'
