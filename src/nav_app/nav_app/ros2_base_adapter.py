"""Standard ROS 2 Twist/Odometry adapter. Launched explicitly on a configured base.

ack_sequence acknowledges software receipt, NOT a hardware command acknowledgement.
stop_confirmed is based on fresh base Odometry, never the navigation/LIO estimate.
The base must also enforce a command watchdog if this computer/link fails.
"""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class ROS2BaseAdapter(Node):
    def __init__(self,**kwargs):
        super().__init__('nav_ros2_base_adapter',**kwargs)
        defaults={'planner_topic':'/nav/planner_cmd_vel','cmd_vel_topic':'/cmd_vel',
                  'base_odom_topic':'/base/odom','enable_topic':'/base/motion_enabled',
                  'base_frame':'base_link','timeout_s':.3}
        for name,value in defaults.items():self.declare_parameter(name,value)
        p=lambda name:self.get_parameter(name).value
        if not .1<=p('timeout_s')<=1:raise ValueError('timeout_s must be 0.1..1')
        if p('cmd_vel_topic')==p('planner_topic'):raise ValueError('Planner and base topics must differ')
        if p('base_odom_topic') in ('/aft_mapped_to_init','/lio/odom'):raise ValueError('Use base feedback, not LIO')
        self.timeout=p('timeout_s');self.frame=p('base_frame');self.last_command=None
        self.last_odom=None;self.last_enable=None;self.enabled=False;self.measured=None
        self.sequence=0;self.velocity=(0.,0.,0.);self.fault=False;self.last_tick=time.monotonic()
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output=self.create_publisher(Twist,p('cmd_vel_topic'),10)
        self.request=self.create_publisher(TwistStamped,'/nav/velocity_request',qos)
        self.feedback_pub=self.create_publisher(String,'/nav/base_feedback',qos)
        self.create_subscription(Twist,p('planner_topic'),self.planner,qos)
        self.create_subscription(String,'/nav/base_command',self.command,qos)
        self.create_subscription(Odometry,p('base_odom_topic'),self.odometry,qos)
        self.create_subscription(Bool,p('enable_topic'),self.enable,qos)
        self.create_timer(.025,self.tick)

    def age_ok(self,stamp):
        return type(stamp)==int and -.05<=(self.get_clock().now().nanoseconds-stamp)/1e9<=self.timeout

    def planner(self,message):
        # Twist has no source timestamp. Depth-one QoS and a receipt watchdog are
        # the available guarantee; hardware adapters can use TwistStamped upstream.
        request=TwistStamped();request.header.stamp=self.get_clock().now().to_msg()
        request.header.frame_id=self.frame;request.twist=message;self.request.publish(request)

    def command(self,message):
        try:
            if len(message.data)>2048:raise ValueError()
            row=json.loads(message.data)
            if not isinstance(row,dict):raise ValueError()
            velocity=row['velocity'];sequence=row['sequence']
            if (row.get('schema')!=1 or row.get('frame_id')!=self.frame or not self.age_ok(row['stamp_ns'])
                or type(sequence)!=int or sequence<=self.sequence or len(velocity)!=3
                or not all(type(v) in (int,float) and math.isfinite(v) for v in velocity)
                or math.hypot(*velocity[:2])>.500001 or abs(velocity[2])>1.000001):raise ValueError()
            self.sequence=sequence;self.velocity=tuple(velocity);self.last_command=time.monotonic()
        except (ValueError,KeyError,TypeError):
            self.fault=True;self.velocity=(0.,0.,0.);self.zero()

    def enable(self,message):
        self.enabled=message.data is True;self.last_enable=time.monotonic()
        if not self.enabled:self.zero()

    def odometry(self,message):
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        v=message.twist.twist;values=(v.linear.x,v.linear.y,v.angular.z)
        if message.child_frame_id!=self.frame or not self.age_ok(stamp) or not all(math.isfinite(x) for x in values):
            self.last_odom=None;self.zero();return
        self.last_odom=time.monotonic();self.measured=values

    def zero(self):self.output.publish(Twist())

    def tick(self):
        now=time.monotonic();gap=now-self.last_tick;self.last_tick=now
        if not 0<=gap<=self.timeout:self.fault=True
        fresh=lambda stamp:stamp is not None and 0<=now-stamp<=self.timeout
        connected=fresh(self.last_odom) and not self.fault
        enabled=self.enabled and fresh(self.last_enable)
        command_fresh=fresh(self.last_command)
        # A timed-out guard is latched. Restart explicitly to re-establish sequence
        # ownership; newly arriving commands alone must not resume movement.
        if self.last_command is not None and not command_fresh:self.fault=True;connected=False
        msg=Twist()
        if connected and enabled and command_fresh:
            msg.linear.x,msg.linear.y,msg.angular.z=self.velocity
        self.output.publish(msg)
        if self.measured is not None:
            feedback=dict(schema=1,stamp_ns=self.get_clock().now().nanoseconds,connected=connected,
                estop_released=enabled,ack_sequence=self.sequence,measured_velocity=self.measured,
                acknowledgement_source='software_adapter',velocity_source='base_odometry')
            self.feedback_pub.publish(String(data=json.dumps(feedback,allow_nan=False)))


def main(args=None):
    rclpy.init(args=args);node=ROS2BaseAdapter()
    try:rclpy.spin(node)
    finally:node.zero();node.destroy_node();rclpy.shutdown()
