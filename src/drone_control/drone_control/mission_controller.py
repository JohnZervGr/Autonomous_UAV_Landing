import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped

from std_msgs.msg import Bool




from enum import Enum


class FlightState(Enum):
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    MISSION = "mission"


class FSMController(Node):

    def __init__(self):
        super().__init__('fsm_controller')

        self.state = FlightState.PREFLIGHT
        self.current_state = State()
        self.extended_state = ExtendedState()

        self.setpoint_counter = 0
        self.target_altitude = 2.0
        self.current_altitude = 0.0

        self.marker_visible = False
        self.tracking_error = Vector3Stamped()

        # Subscribers
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            10
        )

        self.create_subscription(Vector3Stamped, '/controller/tracking_error', self.tracking_error_cb, 10)

        self.create_subscription(Bool,'/controller/target_visible', self.target_visible_cb, 10)

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        # Services
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Timer
        self.timer = self.create_timer(0.1, self.run_fsm)
    
    def tracking_error_cb(self, msg):
        self.get_logger().info(f"Tracking error: x={msg.vector.x:.2f}, y={msg.vector.y:.2f}, z={msg.vector.z:.2f}")
        self.tracking_error = msg
        return
    

    def target_visible_cb(self, msg):
        self.marker_visible = msg.data
        return

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_altitude = msg.pose.position.z

    def publish_velocity(self, vx, vy, vz):
        twist = TwistStamped()
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        self.vel_pub.publish(twist)

    def publish_setpoint(self, z):
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = z
        self.pose_pub.publish(pose)

    def run_fsm(self):

        # Always publish something (OFFBOARD requirement)
        if self.state == FlightState.PREFLIGHT:
            self.publish_setpoint(0.0)
        elif self.state == FlightState.TAKEOFF:
            self.publish_setpoint(self.target_altitude)
        elif self.state == FlightState.MISSION:
            self.get_logger().info(f"Marker visible: {self.marker_visible}")
            if self.marker_visible:
                self.publish_velocity(*self.compute_velocity())
            else:
                self.publish_velocity(0.0, 0.0, 0.0)

        # =====================
        # PREFLIGHT STATE
        # =====================
        if self.state == FlightState.PREFLIGHT:

            if not self.current_state.connected:
                return

            self.setpoint_counter += 1

            # Send some initial setpoints first
            if self.setpoint_counter < 20:
                return

            # Arm
            if not self.current_state.armed:
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self.get_logger().info("Arming...")
                return

            # Set OFFBOARD
            if self.current_state.mode != "OFFBOARD":
                req = SetMode.Request()
                req.custom_mode = "OFFBOARD"
                self.mode_client.call_async(req)
                self.get_logger().info("Switching to OFFBOARD...")
                return

            # Transition
            self.get_logger().info("Preflight complete → TAKEOFF")
            self.state = FlightState.TAKEOFF

        # =====================
        # TAKEOFF STATE
        # =====================
        elif self.state == FlightState.TAKEOFF:

            if self.marker_visible:
                self.get_logger().info("ArUco marker detected → MISSION")
                self.state = FlightState.MISSION
        
        elif self.state == FlightState.MISSION:
            if not self.marker_visible:
                self.get_logger().info("Marker lost → TAKEOFF")
                self.state = FlightState.TAKEOFF

    def compute_velocity(self):
        kx = 0.3
        ky = 0.3
        kz = 0.2

        max_xy = 0.5
        max_z = 0.3

        ex = self.tracking_error.vector.x
        ey = self.tracking_error.vector.y
        ez = self.tracking_error.vector.z

        vx = self.clamp(kx * ex, max_xy)
        vy = self.clamp(ky * ey, max_xy)
        vz = self.clamp(kz * ez, max_z)

        return vx, vy, vz
    
    def clamp(self, val, max_val):
        return max(min(val, max_val), -max_val)

def main(args=None):
    rclpy.init(args=args)
    node = FSMController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()