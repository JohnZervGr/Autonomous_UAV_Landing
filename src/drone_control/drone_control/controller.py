import math
import rclpy
from rclpy.node import Node

from enum import Enum

from mavros_msgs.msg import State, ExtendedState
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Bool



class FlightState(Enum):
    PREFLIGHT = "preflight"
    MISSION = "mission"


class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.state = FlightState.PREFLIGHT

        self.transitions = {
            (FlightState.PREFLIGHT, "takeoff_complete"): (FlightState.MISSION, self.on_takeoff_complete),
        }

        # Vehicle status
        self.current_mode = "UNKNOWN"
        self.is_armed = False
        self.is_connected = False
        self.extended_state = ExtendedState()
        self.local_pose = PoseStamped()

        # Target takeoff setpoint
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 3.0
        self.pos_tolerance = 0.20

        # Offboard / arming sequencing
        self.initial_setpoint_count = 0
        self.required_initial_setpoints = 100  # 10 seconds at 10 Hz
        self.offboard_requested = False
        self.arm_requested = False
        self.offboard_request_pending = False
        self.arm_request_pending = False

        ####################### SERVICES #################################
        self.arm_srv = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_srv = self.create_client(SetMode, "/mavros/set_mode")

        ####################### SUBSCRIBERS ###############################
        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10
        )

        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self.extended_state_callback,
            10
        )

        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_pose_cb,
            10
        )

        self.marker_visible = self.create_subscription(
            Bool,
            '/controller/target_visible',
            self.marker_visible_cb,
            10
        )

        self.error_pub = self.create_publisher(
            Vector3Stamped,
            '/controller/position_error',
            self.error_cb,
            10
        )

        ####################### PUBLISHERS ###############################
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10
        )

        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.get_logger().info("Controller node initialized, waiting for FCU connection...")

    def on_takeoff_complete(self):
        self.get_logger().info("Takeoff complete, transitioning to MISSION")

    def error_cb(self, msg: Vector3Stamped):
        # This callback can be used to log or process the position error if needed
        pass

    def state_callback(self, msg: State):
        self.current_mode = msg.mode
        self.is_armed = msg.armed
        self.is_connected = msg.connected

    def extended_state_callback(self, msg: ExtendedState):
        self.extended_state = msg

    def local_pose_cb(self, msg: PoseStamped):
        self.local_pose = msg

    def marker_visible_cb(self, msg: Bool):
        self.marker_visible = msg

    def trigger(self, event: str):
        key = (self.state, event)
        if key not in self.transitions:
            self.get_logger().warn(f"Ignored event '{event}' in state '{self.state.name}'")
            return self.state

        next_state, action = self.transitions[key]

        self.get_logger().info(
            f"Transition: {self.state.name} --({event})-> {next_state.name}"
        )

        action()
        self.state = next_state
        return self.state

    def publish_target_setpoint(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.target_x
        pose.pose.position.y = self.target_y
        pose.pose.position.z = self.target_z
        self.setpoint_pub.publish(pose)

    def request_offboard_mode(self):
        if self.offboard_request_pending or self.current_mode == "OFFBOARD":
            return

        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"

        future = self.mode_srv.call_async(req)
        future.add_done_callback(self.offboard_response_cb)
        self.offboard_request_pending = True

        self.get_logger().info("Requesting OFFBOARD mode...")

    def offboard_response_cb(self, future):
        self.offboard_request_pending = False
        try:
            res = future.result()
            if res.mode_sent:
                self.offboard_requested = True
                self.get_logger().info("OFFBOARD mode request accepted")
            else:
                self.get_logger().warn("OFFBOARD mode request rejected")
        except Exception as e:
            self.get_logger().error(f"OFFBOARD mode service call failed: {e}")

    def request_arm(self):
        if self.arm_request_pending or self.is_armed:
            return

        req = CommandBool.Request()
        req.value = True

        future = self.arm_srv.call_async(req)
        future.add_done_callback(self.arm_response_cb)
        self.arm_request_pending = True

        self.get_logger().info("Requesting arm...")

    def arm_response_cb(self, future):
        self.arm_request_pending = False
        try:
            res = future.result()
            if res.success:
                self.arm_requested = True
                self.get_logger().info("Arm request accepted")
            else:
                self.get_logger().warn(f"Arm request rejected, result={res.result}")
        except Exception as e:
            self.get_logger().error(f"Arm service call failed: {e}")

    def position_reached(self) -> bool:
        dx = self.local_pose.pose.position.x - self.target_x
        dy = self.local_pose.pose.position.y - self.target_y
        dz = self.local_pose.pose.position.z - self.target_z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dist <= self.pos_tolerance

    def control_loop(self):
        # Always publish the target setpoint in Offboard workflow
        self.publish_target_setpoint()

        if self.state == FlightState.PREFLIGHT:
            self.handle_preflight()

        elif self.state == FlightState.MISSION:
            self.handle_mission()

    def handle_preflight(self):
        if not self.is_connected:
            self.get_logger().debug("Waiting for FCU connection...")
            return

        # PX4 requires setpoints before OFFBOARD
        if self.initial_setpoint_count < self.required_initial_setpoints:
            self.initial_setpoint_count += 1
            if self.initial_setpoint_count == 1:
                self.get_logger().info("Streaming initial setpoints before OFFBOARD...")
            return

        # Request OFFBOARD once
        if self.current_mode != "OFFBOARD":
            self.request_offboard_mode()
            return

        # Request arm once OFFBOARD is active
        if not self.is_armed:
            self.request_arm()
            return

        # Once OFFBOARD + armed, keep publishing target until reached
        if self.position_reached():
            self.trigger("takeoff_complete")
        else:
            z = self.local_pose.pose.position.z
            self.get_logger().debug(f"Climbing... current z = {z:.2f}")

    def handle_mission(self):
        # Keep holding the reached position for now
        self.publish_target_setpoint()

        # You can replace this later with mission logic
        if self.extended_state.landed_state == ExtendedState.LANDED_STATE_IN_AIR:
            self.get_logger().debug("In MISSION and airborne")

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()