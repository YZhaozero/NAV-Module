"""Manual-only velocity requests; no planner output is connected to this path."""
import math
import time


ZERO = (0., 0., 0.)


def velocity(chassis, forward, left, turn, linear, angular):
    if chassis not in ("ackermann", "omni", "differential"):
        raise ValueError("请先选择实际底盘类型")
    scale = max(1., math.hypot(forward, left)) if chassis == "omni" else 1.
    vx = forward*linear/scale
    vy = left*linear/scale if chassis == "omni" else 0.
    # Ackermann steering follows travel direction and cannot rotate at rest.
    wz = turn*angular*forward if chassis == "ackermann" else turn*angular
    if chassis == "ackermann" and linear == 0:
        wz = 0.
    return vx, vy, wz


def joystick_input(sample, now, chassis, axes, deadman, deadzone=.12, signs=(1, 1, 1), centers=()):
    if sample is None or not 0 <= now-sample[2] <= .3:
        raise ValueError("手柄数据中断，请检查手柄连接和驱动")
    values, buttons, _stamp = sample
    required = (axes[0], axes[2]) if chassis != "omni" else axes
    if any(index < 0 or index >= len(values) for index in required):
        raise ValueError("手柄轴编号超出范围，请按实时输入调整映射")
    if deadman < 0 or deadman >= len(buttons) or buttons[deadman] not in (0, 1):
        raise ValueError("手柄保持键编号无效")
    if any(not math.isfinite(values[index]) for index in required):
        raise ValueError("手柄包含非有限轴值")
    if centers and len(centers) != len(values):
        raise ValueError("手柄轴数量与标定不一致，请重新标定")
    mapped = [(values[index]-(centers[index] if centers else 0.))*signs[i]
              if i != 1 or chassis == "omni" else 0. for i, index in enumerate(axes)]
    mapped = tuple(0. if abs(value) < deadzone else max(-1., min(1., value)) for value in mapped)
    return mapped, bool(buttons[deadman])


class ManualDrive:
    def __init__(self, busy, clock=time.monotonic):
        self.busy, self.clock = busy, clock
        self.transport = None
        self.chassis = None
        self.source = "screen"
        self.linear, self.angular = .15, .30
        self.axes, self.deadman = (1, 3, 0), 4
        self.signs, self.centers, self.joy = (1, 1, 1), (), None
        self.strafe = False
        self.screen = (ZERO, False)
        self.armed = False
        self.message = "先连接 Agent，再选择车型并允许遥控"
        self.pending = self.stop_pending = None
        self.pending_at = self.stop_at = 0.
        self.last_send = -1.
        self.last_command = ZERO

    def health(self):
        if self.transport is None:
            return dict(ready=False, estop_released=False, mode=0,
                        measured=None, reason="未连接厂家 Agent")
        return self.transport.health(self.clock())

    def arm(self):
        health = self.health()
        reason = (self.busy() or
                  ("请选择实际底盘类型" if self.chassis is None else "") or
                  (health["reason"] if not health["ready"] else "") or
                  ("车身急停已触发" if not health["estop_released"] else ""))
        if reason:
            self.message = reason
            return False
        try:
            values, held = self.input()
        except ValueError as error:
            self.message = str(error)
            return False
        if held or any(values):
            self.message = "请先将摇杆回中并松开保持键，再启用遥控"
            return False
        if self.pending is not None or self.stop_pending is not None:
            self.message = "等待上一条速度请求结束"
            return False
        self.armed, self.message = True, "遥控已允许；按住摇杆或手柄保持键移动"
        return True

    def input(self):
        if self.source == "gamepad":
            return joystick_input(self.joy, self.clock(), self.chassis, self.axes, self.deadman, signs=self.signs, centers=self.centers)
        return self.screen

    def screen_input(self, x, forward, held):
        self.screen = ((forward, -x if self.strafe else 0., 0. if self.strafe else -x), held)
        if not held and self.armed:
            self.zero()

    def zero(self):
        if self.transport is None or self.stop_pending is not None:
            return
        try:
            self.stop_pending = self.transport.send(ZERO)
            self.stop_at = self.clock()
            self.last_command = ZERO
        except RuntimeError as error:
            self.message = "零速请求未确认："+str(error)

    def stop(self, reason="已停止遥控", force=False):
        owned = self.armed or any(self.last_command)
        self.armed = False
        self.screen = (ZERO, False)
        self.message = reason
        if owned or force:
            self.zero()

    def tick(self):
        now = self.clock()
        if self.transport is None:
            return
        self.transport.poll(now)
        for field, time_field in (("pending", "pending_at"), ("stop_pending", "stop_at")):
            stamp = getattr(self, time_field)
            future = getattr(self, field)
            if future is None:
                continue
            if future.done():
                setattr(self, field, None)
                try:
                    response = future.result()
                    if not response.ok:
                        raise RuntimeError(response.msg)
                except (RuntimeError, ValueError) as error:
                    self.stop("速度请求被拒绝："+str(error))
            elif now-stamp > .5:
                self.transport.cancel(future)
                setattr(self, field, None)
                self.stop("速度服务响应超时，遥控已取消")
        if not self.armed:
            return
        health = self.health()
        reason = (self.busy() or (health["reason"] if not health["ready"] else "") or
                  ("车身急停已触发" if not health["estop_released"] else ""))
        if reason:
            self.stop(reason)
            return
        try:
            values, held = self.input()
        except ValueError as error:
            self.stop(str(error))
            return
        command = velocity(self.chassis, *values, self.linear, self.angular) if held else ZERO
        # Zero requests bypass an outstanding movement request on release.
        if not held and any(self.last_command):
            self.zero()
        if self.pending is None and self.stop_pending is None and now-self.last_send >= .05:
            try:
                self.pending = self.transport.send(command)
                self.pending_at = self.last_send = now
                self.last_command = command
            except RuntimeError as error:
                self.stop(str(error))


class WooshTeleopRos:
    """Lazy vendor bindings: connecting only polls status and subscribes to Joy."""
    def __init__(self, node, namespace="/woosh_robot", clock=time.monotonic):
        from woosh_robot_msgs.srv import Twist, Mode, DeviceState, PoseSpeed
        from woosh_robot_msgs.msg import DeviceStateHardwareBit
        self.node, self.clock = node, clock
        self.types = dict(Twist=Twist, Mode=Mode, DeviceState=DeviceState,
                          PoseSpeed=PoseSpeed)
        self.clients = {name: node.create_client(cls, namespace+"/robot/"+name)
                        for name, cls in self.types.items()}
        self.emergency_mask = DeviceStateHardwareBit.K_EMG_BTN
        self.samples, self.reads, self.errors = {}, {}, {}
        self.last_poll = -1.
    def poll(self, now):
        for name, (future, started) in list(self.reads.items()):
            if future.done():
                del self.reads[name]
                try:
                    value = future.result().info
                    if name == "Mode" and value.ctrl.value not in (0, 1, 2, 3):
                        raise ValueError("未知控制模式："+str(value.ctrl.value))
                    if name == "PoseSpeed" and (not value.has_field & value.TWIST_FIELD_SET or
                            not all(math.isfinite(v) for v in (value.twist.linear, value.twist.angular))):
                        raise ValueError("速度反馈缺失或无效")
                    self.samples[name] = (now, value)
                    self.errors.pop(name, None)
                except (RuntimeError, ValueError) as error:
                    self.errors[name] = str(error)
            elif now-started > .8:
                self.clients[name].remove_pending_request(future)
                del self.reads[name]
                self.errors[name] = name+" 状态请求超时"
        if now-self.last_poll >= .2:
            self.last_poll = now
            for name in ("Mode", "DeviceState", "PoseSpeed"):
                if name not in self.reads and self.clients[name].service_is_ready():
                    self.reads[name] = (self.clients[name].call_async(self.types[name].Request()), now)

    def health(self, now):
        reason = ""
        if not self.clients["Twist"].service_is_ready():
            reason = "等待 /woosh_robot/robot/Twist 服务"
        elif self.errors:
            reason = next(iter(self.errors.values()))
        else:
            for name in ("Mode", "DeviceState", "PoseSpeed"):
                if name not in self.samples or not 0 <= now-self.samples[name][0] <= 1.:
                    reason = "等待有效状态反馈："+name
                    break
        if reason:
            return dict(ready=False, estop_released=False, mode=0, measured=None, reason=reason)
        mode = self.samples["Mode"][1].ctrl.value
        device = self.samples["DeviceState"][1]
        speed = self.samples["PoseSpeed"][1].twist
        return dict(ready=True,
                    estop_released=not bool(device.hardware & self.emergency_mask), mode=mode,
                    measured=(speed.linear, speed.angular), reason="Agent 状态反馈正常")

    def send(self, values):
        client = self.clients["Twist"]
        if not client.service_is_ready():
            raise RuntimeError("厂家速度服务不可用")
        request = self.types["Twist"].Request()
        request.arg.linear, request.arg.linear_y, request.arg.angular = values
        return client.call_async(request)

    def cancel(self, future):
        self.clients["Twist"].remove_pending_request(future)

    def close(self):
        for client in self.clients.values():
            self.node.destroy_client(client)
