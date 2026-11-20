"""Optional base adapter boundary. Never auto-started by NAV.

The real adapter consumes /nav/base_command and returns /nav/base_feedback
with acknowledgement and measured velocity. Default state is disarmed.
"""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool,String
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from .features.motion_guard import MotionGuard
from .features.localization_health import LocalizationHealth


class MotionGuardNode(Node):
    def __init__(self):
        super().__init__('nav_motion_guard')
        for name,value in [('max_speed_mps',.3),('max_yaw_rps',.5),('acceleration',.4),('timeout_s',.3)]:self.declare_parameter(name,value)
        self.guard=MotionGuard(max_speed=self.get_parameter('max_speed_mps').value,max_yaw=self.get_parameter('max_yaw_rps').value,
                               acceleration=self.get_parameter('acceleration').value,timeout=self.get_parameter('timeout_s').value)
        self.base_reason = ''
        self.base_feedback_stamp = 0
        self.health=LocalizationHealth();qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.command=self.create_publisher(String,'/nav/base_command',qos)
        self.status=self.create_publisher(String,'/nav/motion_status',qos)
        self.create_subscription(Bool,'/nav/motion_permit',lambda m:self.guard.permit(m.data),qos)
        self.create_subscription(String,'/localizer/localizer_node/quality',lambda m:self.health.update(m.data),qos)
        self.create_subscription(TwistStamped,'/nav/velocity_request',self.velocity,qos)
        self.create_subscription(String,'/nav/base_feedback',self.feedback,qos)
        self.create_service(SetBool,'/nav/arm_motion',self.arm)
        self.create_timer(.05,self.tick)

    def age_ok(self,stamp):
        return type(stamp)==int and -.05<=(self.get_clock().now().nanoseconds-stamp)/1e9<=.25

    def velocity(self,m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if m.header.frame_id!='base_link' or not self.age_ok(stamp) or any(v!=0 for v in (m.twist.linear.z,m.twist.angular.x,m.twist.angular.y)):
            self.guard.disarm('invalid_command_frame_or_stamp');return
        self.guard.velocity((m.twist.linear.x,m.twist.linear.y,m.twist.angular.z))

    def feedback(self,m):
        try:
            if len(m.data)>2048:raise ValueError()
            row=json.loads(m.data)
            if not isinstance(row,dict):raise ValueError()
            if row.get('schema')!=1 or not self.age_ok(row['stamp_ns']):raise ValueError()
            reason = row.get('base_reason', '')
            if not isinstance(reason, str):raise ValueError()
            self.base_reason, self.base_feedback_stamp = reason, row['stamp_ns']
            self.guard.feedback(row['connected'],row['estop_released'],row['ack_sequence'],
                                row['measured_velocity'], row.get('stop_pending', False))
        except (ValueError,KeyError,TypeError):self.guard.disarm('invalid_base_feedback')

    def arm(self,request,response):
        self.guard.localization(self.health.valid(self.get_clock().now().nanoseconds))
        if request.data:response.success=self.guard.arm()
        else:self.guard.disarm();response.success=True
        response.message=self.guard.reason;return response

    def tick(self):
        self.guard.localization(self.health.valid(self.get_clock().now().nanoseconds))
        state=self.guard.tick();state['stamp_ns']=self.get_clock().now().nanoseconds;state['frame_id']='base_link'
        self.command.publish(String(data=json.dumps(state,allow_nan=False)))
        state['base_reason'] = (self.base_reason if self.age_ok(self.base_feedback_stamp)
                                else '等待底盘适配器反馈')
        self.status.publish(String(data=json.dumps(state,allow_nan=False)))


def main(args=None):
    rclpy.init(args=args);node=MotionGuardNode()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    finally:
        node.guard.disarm('shutdown')
        if rclpy.ok():node.tick()
        node.destroy_node();rclpy.try_shutdown()
