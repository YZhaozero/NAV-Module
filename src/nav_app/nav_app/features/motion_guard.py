"""Platform-independent fail-closed velocity gate; feedback must come from the base.

Navigation action cancellation is never treated as evidence of a physical stop.
This gate is not a hardware emergency stop or a substitute for collision checking.
"""
import math
import time


class MotionGuard:
    def __init__(self,clock=time.monotonic,max_speed=.3,max_yaw=.5,acceleration=.4,timeout=.3):
        if any(not math.isfinite(x) or x<=0 for x in (max_speed,max_yaw,acceleration,timeout)) or timeout>1:
            raise ValueError('Invalid bounded motion limits')
        self.clock=clock;self.max_speed=min(max_speed,.5);self.max_yaw=min(max_yaw,1.)
        self.acceleration=min(acceleration,1.);self.timeout=timeout
        self.armed=False;self.reason='disarmed';self.permit_time=None;self.command_time=None;self.feedback_time=None
        self.command=(0.,0.,0.);self.measured=None;self.stop_pending=False;self.sequence=0;self.ack=0;self.sent=(0.,0.,0.)
        self.last_tick=clock();self.healthy=False;self.connected=False;self.estop_released=False

    def disarm(self,reason='disarmed'):
        self.armed=False;self.reason=reason;self.command_time=None;self.command=(0.,0.,0.)

    def permit(self,enabled):
        if enabled:self.permit_time=self.clock()
        else:self.permit_time=None;self.disarm('operator_or_navigation_stop')

    def localization(self,valid):
        self.healthy=bool(valid)
        if not self.healthy:self.disarm('localization_unavailable')

    def feedback(self,connected,estop_released,ack,velocity,stop_pending=False):
        if (type(ack)!=int or not 0<=ack<=self.sequence or not isinstance(velocity,(list,tuple)) or len(velocity)!=3
                or not all(type(x) in (int,float) and math.isfinite(x) for x in velocity)):
            self.disarm('invalid_base_feedback');self.feedback_time=None;return
        self.feedback_time=self.clock();self.connected=connected is True;self.estop_released=estop_released is True
        self.ack=ack;self.measured=tuple(velocity);self.stop_pending=stop_pending
        if not self.connected or not self.estop_released:self.disarm('base_unavailable')

    def unready_reason(self):
        now = self.clock()
        if not self.healthy:
            return "localization_unavailable"
        if self.feedback_time is None or not 0 <= now - self.feedback_time <= self.timeout:
            return "base_feedback_timeout"
        if not self.connected or not self.estop_released:
            return "base_unavailable"
        if self.sequence - self.ack > 6:
            return "base_ack_lag"
        if self.permit_time is None or not 0 <= now - self.permit_time <= self.timeout:
            return "permit_timeout"
        return ""

    def ready(self):
        return not self.unready_reason()

    def arm(self):
        if not self.ready():self.disarm('fresh_permit_localization_and_base_feedback_required');return False
        self.armed=True;self.reason='armed';self.command_time=None;return True

    def velocity(self,values):
        if len(values)!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in values):
            self.disarm('invalid_velocity');return
        self.command=tuple(values);self.command_time=self.clock()

    def tick(self):
        now=self.clock();dt=now-self.last_tick;self.last_tick=now
        if dt<0 or dt>self.timeout:self.disarm('control_clock_or_scheduler_gap')
        if self.armed:
            reason = self.unready_reason()
            if reason:
                self.disarm(reason)
        if self.armed and self.command_time is not None and not 0<=now-self.command_time<=self.timeout:self.disarm('command_timeout')
        target=(0.,0.,0.)
        if self.armed and self.command_time is not None:
            x,y,yaw=self.command;length=math.hypot(x,y);scale=min(1.,self.max_speed/max(length,1e-12))
            target=(x*scale,y*scale,max(-self.max_yaw,min(self.max_yaw,yaw)))
            # Acceleration-limited normal commands. Fault/stop is immediate zero.
            step=self.acceleration*max(0.,min(dt,.1))
            target=tuple(old+max(-step,min(step,new-old)) for old,new in zip(self.sent,target))
            speed=math.hypot(*target[:2])
            if speed>self.max_speed:target=(target[0]*self.max_speed/speed,target[1]*self.max_speed/speed,target[2])
        self.sent=target;self.sequence+=1
        stopped=bool(self.connected and not self.stop_pending and self.feedback_time is not None and 0<=now-self.feedback_time<=self.timeout
                     and self.measured is not None and math.hypot(*self.measured[:2])<.02 and abs(self.measured[2])<.03)
        return dict(schema=1,sequence=self.sequence,velocity=list(target),armed=self.armed,reason=self.reason,
                    stop_confirmed=stopped and not self.armed,
                    ready_to_start=stopped and not self.armed and self.estop_released and self.healthy,
                    measured_velocity=self.measured,
                    base_connected=bool(self.connected and self.feedback_time is not None
                        and 0<=now-self.feedback_time<=self.timeout))
