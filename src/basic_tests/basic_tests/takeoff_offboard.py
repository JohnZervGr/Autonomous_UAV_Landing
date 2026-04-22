#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class TakeoffOffboard(Node):
    """
    PX4 OFFBOARD takeoff using MAVROS (ROS 2):
      - Wait for FCU connection and local pose
      - Stream position setpoints (required)
      - Switch to OFFBOARD
      - Arm
      - Hold target altitude
    """

    def __init__(self):
        super().__init__("takeoff_offboard")

        # Parameters
        self.declare_parameter("takeoff_height", 2.0)     # meters above current z
        self.declare_parameter("setpoint_rate_hz", 20.0)  # must be > 2 Hz for PX4
        self.declare_parameter("pre_stream_sec", 2.0)     # stream before OFFBOARD
        self.declare_parameter("service_timeout_sec", 5.0)

        self.takeoff_height = float(self.get_parameter("takeoff_height").value)
        self.rate_hz = float(self.get_parameter("setpoint_rate_hz").value)
        self.pre_stream_sec = float(self.get_parameter("pre_stream_sec").value)
        self.service_timeout = float(self.get_parameter("service_timeout_sec").value)

        # State from MAVROS
        self.mavros_state = State()
        self.have_state = False
        self.have_pose = False
        self.current_pose = PoseStamped()

        # Target pose (set once we have pose)
        self.target_pose = PoseStamped()
        self.target_set = False

        # Timing / progress flags
        self.start_time = time.time()
        self.offboard_requested = False
        self.armed_requested = False
        self.last_service_call = 0.0

        # Subscribers
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, 10
        )

        # Publisher (position setpoint)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )

        # Service clients
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.get_logger().info("Waiting for MAVROS services...")
        if not self.arm_cli.wait_for_service(timeout_sec=self.service_timeout):
            raise RuntimeError("Service /mavros/cmd/arming not available")
        if not self.mode_cli.wait_for_service(timeout_sec=self.service_timeout):
            raise RuntimeError("Service /mavros/set_mode not available")
        self.get_logger().info("Services ready. Waiting for /mavros/state connected + local pose...")

        # Timer loop
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.loop)

    def state_cb(self, msg: State):
        self.mavros_state = msg
        self.have_state = True

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg
        self.have_pose = True

        if not self.target_set:
            # Target: same XY, Z + takeoff_height, keep current orientation
            self.target_pose.header.frame_id = msg.header.frame_id
            self.target_pose.pose.position.x = msg.pose.position.x
            self.target_pose.pose.position.y = msg.pose.position.y
            self.target_pose.pose.position.z = msg.pose.position.z + self.takeoff_height
            self.target_pose.pose.orientation = msg.pose.orientation

            self.target_set = True
            self.get_logger().info(
                f"Target set: x={self.target_pose.pose.position.x:.2f}, "
                f"y={self.target_pose.pose.position.y:.2f}, "
                f"z={self.target_pose.pose.position.z:.2f}"
            )

    def loop(self):
        if not (self.have_state and self.have_pose and self.target_set):
            return

        if not self.mavros_state.connected:
            # Don’t spam logs at 20Hz
            if int(time.time()) % 2 == 0:
                self.get_logger().warn("Not connected to FCU yet (mavros/state.connected=false)")
            return

        # Always publish setpoints (PX4 requires continuous stream)
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.target_pose)

        now = time.time()

        # 1) Pre-stream setpoints for a bit before requesting OFFBOARD
        if (now - self.start_time) < self.pre_stream_sec:
            return

        # Rate-limit service calls
        if now - self.last_service_call < 0.5:
            return

        # 2) Request OFFBOARD if not already in it
        if self.mavros_state.mode != "OFFBOARD" and not self.offboard_requested:
            self.last_service_call = now
            self.request_offboard()
            return

        # If OFFBOARD got rejected earlier, allow retry
        if self.mavros_state.mode != "OFFBOARD" and self.offboard_requested:
            self.offboard_requested = False
            return

        # 3) Arm if in OFFBOARD and not armed
        if self.mavros_state.mode == "OFFBOARD" and not self.mavros_state.armed and not self.armed_requested:
            self.last_service_call = now
            self.request_arm()
            return

        # Done (keep streaming setpoints so it holds position)
        if self.mavros_state.mode == "OFFBOARD" and self.mavros_state.armed:
            # Log once
            if int(now - self.start_time) == int(self.pre_stream_sec) + 1:
                self.get_logger().info("✅ Takeoff sequence active: OFFBOARD + ARMED. Holding target altitude.")

    def request_offboard(self):
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.offboard_requested = True
        fut = self.mode_cli.call_async(req)
        fut.add_done_callback(self._mode_cb)

    def _mode_cb(self, fut):
        try:
            res = fut.result()
            if res and res.mode_sent:
                self.get_logger().info("OFFBOARD mode request sent (accepted by MAVROS).")
            else:
                self.get_logger().warn("OFFBOARD mode request not accepted by MAVROS.")
                self.offboard_requested = False
        except Exception as e:
            self.get_logger().error(f"SetMode failed: {e}")
            self.offboard_requested = False

    def request_arm(self):
        req = CommandBool.Request()
        req.value = True
        self.armed_requested = True
        fut = self.arm_cli.call_async(req)
        fut.add_done_callback(self._arm_cb)

    def _arm_cb(self, fut):
        try:
            res = fut.result()
            if res and res.success:
                self.get_logger().info("Arming accepted by FCU.")
            else:
                self.get_logger().warn(f"Arming rejected by FCU. result={getattr(res,'result',None)}")
                self.armed_requested = False
        except Exception as e:
            self.get_logger().error(f"Arming failed: {e}")
            self.armed_requested = False


def main():
    rclpy.init()
    node = TakeoffOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
