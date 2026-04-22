#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class KeyboardOffboardController(Node):
    def __init__(self):
        super().__init__('keyboard_offboard_controller')

        self.current_state = State()
        self.latest_cmd = Twist()

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_cb,
            10
        )

        self.vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)  # 20 Hz
        self.control_timer = self.create_timer(1.0, self.try_enable_offboard)

        self.mode_request_in_progress = False
        self.arm_request_in_progress = False

    def state_cb(self, msg: State):
        self.current_state = msg

    def cmd_cb(self, msg: Twist):
        self.latest_cmd = msg

    def publish_setpoint(self):
        sp = TwistStamped()
        sp.header.stamp = self.get_clock().now().to_msg()

        sp.twist.linear.x = self.latest_cmd.linear.x
        sp.twist.linear.y = self.latest_cmd.linear.y
        sp.twist.linear.z = self.latest_cmd.linear.z
        sp.twist.angular.z = self.latest_cmd.angular.z

        self.vel_pub.publish(sp)

    def try_enable_offboard(self):
        if not self.current_state.connected:
            self.get_logger().warn('Not connected to FCU yet')
            return

        if self.current_state.mode != 'OFFBOARD' and not self.mode_request_in_progress:
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.mode_client.call_async(req)
            future.add_done_callback(self.mode_response_cb)
            self.mode_request_in_progress = True
            return

        if self.current_state.mode == 'OFFBOARD' and not self.current_state.armed and not self.arm_request_in_progress:
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_response_cb)
            self.arm_request_in_progress = True

    def mode_response_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Set mode response: mode_sent={result.mode_sent}')
        except Exception as e:
            self.get_logger().error(f'Set mode failed: {e}')
        self.mode_request_in_progress = False

    def arm_response_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Arming response: success={result.success}')
        except Exception as e:
            self.get_logger().error(f'Arming failed: {e}')
        self.arm_request_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardOffboardController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()