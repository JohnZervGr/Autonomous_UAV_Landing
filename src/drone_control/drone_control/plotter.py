import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped, TwistStamped

import matplotlib.pyplot as plt
from collections import deque


class DebugPlotter(Node):

    def __init__(self):
        super().__init__('debug_plotter')

        self.max_points = 300

        self.t = deque(maxlen=self.max_points)

        self.ex = deque(maxlen=self.max_points)
        self.ey = deque(maxlen=self.max_points)
        self.ez = deque(maxlen=self.max_points)

        self.vx = deque(maxlen=self.max_points)
        self.vy = deque(maxlen=self.max_points)
        self.vz = deque(maxlen=self.max_points)

        self.start_time = self.get_clock().now()

        self.create_subscription(
            Vector3Stamped,
            '/controller/tracking_error',
            self.error_cb,
            10
        )

        self.create_subscription(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            self.velocity_cb,
            10
        )

        self.last_error = Vector3Stamped()
        self.last_velocity = TwistStamped()

        plt.ion()
        self.fig, (self.ax_err, self.ax_vel) = plt.subplots(2, 1, figsize=(10, 7))

        self.timer = self.create_timer(0.1, self.update_plot)

    def now_sec(self):
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds / 1e9

    def error_cb(self, msg):
        self.last_error = msg

    def velocity_cb(self, msg):
        self.last_velocity = msg

    def update_plot(self):
        time_now = self.now_sec()

        self.t.append(time_now)

        self.ex.append(self.last_error.vector.x)
        self.ey.append(self.last_error.vector.y)
        self.ez.append(self.last_error.vector.z)

        self.vx.append(self.last_velocity.twist.linear.x)
        self.vy.append(self.last_velocity.twist.linear.y)
        self.vz.append(self.last_velocity.twist.linear.z)

        self.ax_err.clear()
        self.ax_vel.clear()

        self.ax_err.plot(self.t, self.ex, label='error x')
        self.ax_err.plot(self.t, self.ey, label='error y')
        self.ax_err.plot(self.t, self.ez, label='error z')
        self.ax_err.set_title('Tracking Error')
        self.ax_err.set_ylabel('error')
        self.ax_err.grid(True)
        self.ax_err.legend()

        self.ax_vel.plot(self.t, self.vx, label='vx command')
        self.ax_vel.plot(self.t, self.vy, label='vy command')
        self.ax_vel.plot(self.t, self.vz, label='vz command')
        self.ax_vel.set_title('Velocity Corrections')
        self.ax_vel.set_xlabel('time [s]')
        self.ax_vel.set_ylabel('velocity [m/s]')
        self.ax_vel.grid(True)
        self.ax_vel.legend()

        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = DebugPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()