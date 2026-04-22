#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool


class ArmDisarm(Node):
    def __init__(self):
        super().__init__("arm_disarm")

        # ROS parameters (set via --ros-args -p ...)
        self.declare_parameter("arm", False)          # True=arm, False=disarm
        self.declare_parameter("timeout_sec", 5.0)    # wait time for service

        self.arm = bool(self.get_parameter("arm").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.cli = self.create_client(CommandBool, "/mavros/cmd/arming")

        action = "ARM" if self.arm else "DISARM"
        self.get_logger().info(f"Request: {action}")

        # Wait for service with timeout
        if not self.cli.wait_for_service(timeout_sec=self.timeout_sec):
            self.get_logger().error(
                f"/mavros/cmd/arming not available after {self.timeout_sec:.1f}s. "
                f"Is mavros_node running?"
            )
            rclpy.shutdown()
            return

        req = CommandBool.Request()
        req.value = self.arm

        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.on_response)

    def on_response(self, future):
        try:
            res = future.result()
            # res.success: bool
            # res.result: uint8 (MAVLink result enum value)
            self.get_logger().info(f"Response: success={res.success}, result={res.result}")

            if res.success:
                self.get_logger().info("Command accepted by FCU.")
            else:
                self.get_logger().warn("Command rejected by FCU.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = ArmDisarm()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
