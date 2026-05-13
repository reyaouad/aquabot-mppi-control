#!/usr/bin/env python3

from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from math import atan2, pi, cos, sin
from scipy.optimize import minimize

me = 'aquabot/base_link'


def clamp(val, high):
    if abs(val) > high:
        return float(val * high / abs(val))
    return float(val)


dt = 0.1


class IMC:
    def __init__(self, axis, node):

        self.axis = axis
        self.i = 0
        self.vd = None
        self.v = None
        self.vp = None
        self.Kd = 0.
        self.Ki = 0.

        if axis == 'x':
            self.vmax = 6.
            self.M = 1000.
            self.kq = 224.
            self.kl = 182
            self.Kp = 1.
            #self.Ki = 0.2
            self.Kd = 0.2
        elif axis == 'y':
            self.vmax = 1.
            self.M = 1000.
            self.kq = 149
            self.kl = 183
            self.Kp = 0.1
            self.Ki = 0.01
        else:
            self.vmax = 1.
            self.M = 200 # 446
            self.kq = 979.0
            self.kl = 1200
            self.Kp = 2.
            self.Ki = 0.1
            self.Kd = 0.3
            self.kq = 0
            self.kl = 0

        self.Kp = node.declare_parameter(f"{axis}.Kp", self.Kp).value
        self.Kd = node.declare_parameter(f"{axis}.Kd", self.Kd).value
        self.Ki = node.declare_parameter(f"{axis}.Ki", self.Ki).value

    def ready(self):
        return self.vd is not None and self.v is not None

    def set_sp(self, sp):
        if self.vd is None:
            self.vd = clamp(sp, self.vmax)
        else:
            self.vd = clamp(0.7*self.vd + 0.3*sp, self.vmax)

    def cmd(self):

        e = self.vd - self.v

        di = e * dt
        if abs(self.Ki*(self.i + di)) < self.vmax:
            self.i += di

        if self.vp is None:
            self.vp = self.v

        ds = self.v - self.vp
        self.vp = self.v

        a = self.Kp * e + self.Ki*self.i - self.Kd*ds/dt

        # to force
        f = self.M*a + self.kl*self.v + self.kq*self.v*abs(self.v)

        return f


x = -3.278
y = 0.6


class Pose:
    def __init__(self, pose):
        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = 2*atan2(pose.orientation.z,pose.orientation.w)


class Thruster:
    def __init__(self, node, side):

        self.angle_pub = node.create_publisher(Float64, f'/aquabot/thrusters/{side}/cmd_pos', 10)
        self.thrust_pub = node.create_publisher(Float64, f'/aquabot/thrusters/{side}/thrust', 10)

    def pub(self, angle, thrust):
        msg = Float64()
        msg.data = clamp(thrust, 5000.)
        self.thrust_pub.publish(msg)
        msg.data = clamp(angle, pi/4)
        self.angle_pub.publish(msg)


class Command(Node):
    def __init__(self):

        super().__init__('control')
        # force use_sim_time
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([param])

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 1)
        self.cmd_t = 0.
        self.vel_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 1)

        self.uni = self.declare_parameter('unicycle', False).value

        self.left = Thruster(self, 'left')
        self.right = Thruster(self, 'right')

        # control gains
        self.imc = {'x': IMC('x', self), 'y': IMC('y', self), 'w': IMC('w',self)}
        self.Kv = self.declare_parameter('Kv', 0.2).value
        self.Kw = self.declare_parameter('Kw', 1.).value
        self.add_on_set_parameters_callback(self.cb_params)

        self.timer = self.create_timer(dt, self.cmd)

    def odom_cb(self, msg: Odometry):
        self.imc['x'].v = msg.twist.twist.linear.x
        self.imc['y'].v = msg.twist.twist.linear.y
        self.imc['w'].v = msg.twist.twist.angular.z
        self.pose = Pose(msg.pose.pose)

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9

    def cmd_cb(self, msg):

        self.cmd_t = self.get_time()
        self.imc['x'].set_sp(msg.linear.x)
        self.imc['y'].set_sp(msg.linear.y)
        self.imc['w'].set_sp(msg.angular.z)

    def cmd(self):

        if not self.imc['x'].ready():
            return

        fxd = self.imc['x'].cmd()
        fyd = self.imc['y'].cmd()
        md = self.imc['w'].cmd()

        if self.uni:

            tl = tr = 0.
            fl = fxd/2 - md/1.2
            fr = fxd/2 + md/1.2
            scale = max(abs(fl/5000),abs(fr/5000))
            if scale > 1.:
                fl /= scale
                fr /= scale
        else:

            def force(u):

                x = -3
                y = 0.6
                fl,fr,tl,tr = u
                fx = fl*cos(tl) + fr*cos(tr)
                fy = fl*sin(tl) + fr*sin(tr)
                m = fl*(x*sin(tl) - y*cos(tl)) + fr*(x*sin(tr) + y*cos(tr))

                return (fx-fxd)**2 + (fy-fyd)**2 + (m-md)**2 + 0.1*tl**2 + 0.1*tr**2

            fl,fr,tl,tr = minimize(force, [0,0,0,0], method='SLSQP',
                    bounds = [(-5000,5000)]*2 + [(-pi/4,pi/4)]*2).x

        self.left.pub(tl,fl)
        self.right.pub(tr,fr)

    def cb_params(self, params):

        accept = False
        for param in params:

            if param.name == 'Kv':
                self.Kv = param.value
            elif param.name == 'Kw':
                self.Kw = param.value

            if '.' not in param.name:
                continue

            axis,gain = param.name.split('.')
            setattr(self.imc[axis], gain, param.value)
            accept = True

        if accept:
            self.get_logger().info(f'Changed {param.name} to {param.value}')

        return SetParametersResult(successful=accept)


rclpy.init()
rclpy.spin(Command())
rclpy.shutdown()
