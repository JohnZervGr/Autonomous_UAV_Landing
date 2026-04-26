#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Bool

class CameraTracker(Node):
    def __init__(self):
        super().__init__('camera_tracker')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('pose_topic', '/aruco/pose')
        self.declare_parameter('detected_topic', '/aruco/detected')
        self.declare_parameter('error_topic', '/controller/tracking_error')
        self.declare_parameter('target_timeout', 0.5)

        self.declare_parameter('desired_x', 0.0)
        self.declare_parameter('desired_y', 0.0)
        self.declare_parameter('desired_z', 2.0)

        self.target_pose_topic = self.get_parameter('pose_topic').value
        self.detected_topic = self.get_parameter('detected_topic').value
        self.tracking_error_topic = self.get_parameter('error_topic').value
        self.target_timeout = self.get_parameter('target_timeout').value


        self.desired_x = self.get_parameter('desired_x').value
        self.desired_y = self.get_parameter('desired_y').value
        self.desired_z = self.get_parameter('desired_z').value

        #start values
        self.marker_detected = False
        self.latest_pose = None
        self.last_detection_time = None

        
        # -------------------------
        # Publisher
        # -------------------------
        self.error_pub = self.create_publisher(
            Vector3Stamped,
            self.tracking_error_topic,
            10
        )

        self.visibility_pub = self.create_publisher(
            Bool,
            '/controller/target_visible',
            10
        )

        # -------------------------
        # Subscribers
        # -------------------------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.pose_callback,
            10
        )

        self.detected_sub = self.create_subscription(
            Bool,
            self.detected_topic,
            self.detected_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
    
    def detected_callback(self, msg: Bool):
        self.get_logger().debug(f"Marker detected: {msg.data}")
        self.marker_detected = msg.data

        if msg.data:
            self.last_detection_time = self.get_clock().now()


    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
    
    def timer_callback(self):
        
        visible = self.is_target_visible()

        visible_msg = Bool()
        visible_msg.data = visible
        self.visibility_pub.publish(visible_msg)
        self.get_logger().info(f"Target visible: {visible} self.latest_pose: {self.latest_pose}")
        if not visible:
            return
        error_msg = Vector3Stamped()
        error_msg.header = self.latest_pose.header

        error_msg.vector.x = self.latest_pose.pose.position.x - self.desired_x
        error_msg.vector.y = self.latest_pose.pose.position.y - self.desired_y
        error_msg.vector.z = self.latest_pose.pose.position.z - self.desired_z

        self.error_pub.publish(error_msg)

    #temp fix will sort out later, should be based on timeouts and not just detection flag
    def is_target_visible(self):
        if self.marker_detected:
            return True
        '''
        if self.last_detection_time is None:
            return False

        if self.latest_pose is None:
            return False'''

        return False
    
def main():
    rclpy.init()
    node = CameraTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()