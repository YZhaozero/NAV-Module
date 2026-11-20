"""One explicit robot route start permits one guard arm attempt; faults latch."""
import json
import time


class MotionSession:
    def __init__(self,client,request_factory,clock=time.monotonic):
        self.client=client;self.request_factory=request_factory;self.clock=clock
        self._sample = ({}, None, "等待底盘状态 /nav/motion_status")
        self.generation=None;self.started=None;self.future=None;self.armed=False

    @property
    def state(self):
        return self._sample[0]

    def update(self,text,ros_now_ns=None):
        try:
            if len(text)>2048:raise ValueError()
            row=json.loads(text)
            if not isinstance(row,dict):raise ValueError()
            if row.get('schema')!=1 or type(row.get('armed'))!=bool or type(row.get('base_connected'))!=bool:raise ValueError()
            age = 0.
            if ros_now_ns is not None:
                if type(row.get("stamp_ns")) is not int:
                    raise ValueError()
                age = (ros_now_ns - row["stamp_ns"]) / 1e9
                if not -.05 <= age <= .3:
                    self._sample = ({}, None, f"底盘状态时间戳异常（相差 {age*1000:.0f} 毫秒）")
                    return
            # A delayed frame only retains the unused part of its 300 ms lifetime.
            self._sample = (row, self.clock() - max(0., age), "")
        except (TypeError, ValueError):
            self._sample = ({}, None, "底盘状态消息格式无效")

    def fresh(self):
        _, stamp, _ = self._sample
        return stamp is not None and 0 <= self.clock() - stamp <= .3

    def unavailable_reason(self):
        state, stamp, error = self._sample
        if error:
            return error
        age = self.clock() - stamp
        if not 0 <= age <= .3:
            return f"底盘状态超时（/nav/motion_status，{age*1000:.0f} 毫秒）"
        if not state["base_connected"]:
            return state.get("base_reason") or "底盘适配器反馈中断"
        if not self.client.service_is_ready():
            return "底盘授权服务 /nav/arm_motion 不可用"
        return ""

    def base_reason(self):
        return self.state.get('base_reason', '') if self.fresh() else ''

    def available(self):
        return not self.unavailable_reason()

    def poll(self,generation,running):
        if not running:
            self.started=None;self.future=None;self.armed=False;return ''
        now=self.clock()
        if generation!=self.generation:
            self.generation=generation;self.started=now;self.future=None;self.armed=False
        if self.armed:
            error = self.unavailable_reason()
            if error or self.state.get("armed") is True:
                return error
            reason = self.state.get("reason", "")
            return {
                "permit_timeout": "导航行驶许可超时",
                "base_feedback_timeout": "底盘适配器反馈超时",
                "base_ack_lag": "底盘指令回执滞后",
                "localization_unavailable": "地图定位质量失效",
                "control_clock_or_scheduler_gap": "底盘安全控制周期超时",
                "command_timeout": "导航速度指令超时",
                "base_unavailable": self.base_reason() or "底盘连接或急停状态异常",
            }.get(reason, f"底盘已撤销行驶许可（{reason}）")
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
