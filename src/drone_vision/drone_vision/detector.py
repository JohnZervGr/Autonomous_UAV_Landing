#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('image_topic', '/gz_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/gz_camera/camera_info')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 1.0)
        self.declare_parameter('camera_frame', 'camera_link')

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_id = self.get_parameter('marker_id').value
        self.dictionary_name = self.get_parameter('dictionary').value
        self.marker_size = self.get_parameter('marker_size').value
        self.camera_frame = self.get_parameter('camera_frame').value


        self.tf_broadcaster = TransformBroadcaster(self)


        # -------------------------
        # OpenCV / ArUco setup
        # -------------------------
        self.bridge = CvBridge()

        # For now we hardcode DICT_4X4_50 because this is known to work
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # -------------------------
        # Camera calibration storage
        # -------------------------
        self.camera_info_received = False
        self.camera_matrix = None
        self.dist_coeffs = None

        # -------------------------
        # Publishers
        # -------------------------
        self.detected_pub = self.create_publisher(
            Bool,
            '/aruco/detected',
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/aruco/pose',
            10
        )

        # -------------------------
        # Subscribers
        # -------------------------
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.get_logger().info(f'Subscribed to image topic: {self.image_topic}')
        self.get_logger().info(f'Subscribed to camera info topic: {self.camera_info_topic}')
        self.get_logger().info(f'Target marker ID: {self.marker_id}')

    # -------------------------------------------------------------------------
    # Camera info handling
    # -------------------------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration data from CameraInfo."""

        if self.camera_info_received:
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

        self.camera_info_received = True

        self.get_logger().info('Camera info received.')

    # -------------------------------------------------------------------------
    # Image conversion
    # -------------------------------------------------------------------------
    def ros_image_to_cv2(self, msg: Image):
        """Convert ROS Image message to OpenCV BGR image."""

        try:
            return self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return None

    # -------------------------------------------------------------------------
    # ArUco detection
    # -------------------------------------------------------------------------
    def detect_target_marker(self, frame):
        """
        Detect ArUco markers in the image.

        Returns:
            detected: True if target marker is found
            corners: detected marker corners
            ids: detected marker IDs
        """

        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is None:
            return False, corners, ids

        ids_list = ids.flatten().tolist()
        return self.marker_id in ids_list, corners, ids

    # -------------------------------------------------------------------------
    # Image callback
    # -------------------------------------------------------------------------
    def image_callback(self, msg: Image):
        """Main image callback: convert image, detect marker, publish result."""

        

        frame = self.ros_image_to_cv2(msg)
        # marker detection
        if frame is None:
            return

        detected, corners, ids = self.detect_target_marker(frame)

        detected_msg = Bool()
        detected_msg.data = detected
        self.detected_pub.publish(detected_msg)

        if detected:
            self.get_logger().debug(f'Target marker detected: {self.marker_id}')
        else:
            self.get_logger().debug('Target marker not detected.')

    
        #pose estimetion        
        if detected and ids is not None:
            self.last_image_stamp = msg.header.stamp
            pose_msg = self.estimate_target_pose(corners, ids)

            if pose_msg is not None:
                self.pose_pub.publish(pose_msg)
                self.publish_tf(pose_msg)

        

    def estimate_target_pose(self, corners, ids):
        if not self.camera_info_received:
            return None

        ids_list = ids.flatten().tolist()

        if self.marker_id not in ids_list:
            return None

        target_index = ids_list.index(self.marker_id)
        target_corners = [corners[target_index]]

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            target_corners,
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )

        tvec = tvecs[0][0]
        rvec = rvecs[0][0]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.last_image_stamp
        pose_msg.header.frame_id = 'camera_link'

        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])

        # Temporary orientation placeholder.
        R, _ = cv2.Rodrigues(rvec)
        Q = self.rotation_to_quaternion(R)

        pose_msg.pose.orientation.x = float(Q[0])
        pose_msg.pose.orientation.y = float(Q[1])
        pose_msg.pose.orientation.z = float(Q[2])
        pose_msg.pose.orientation.w = float(Q[3])

        return pose_msg

    def rotation_to_quaternion(self, R):
        """Convert a 3x3 rotation matrix to quaternion."""

        q = np.zeros(4)

        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s

        return q 
    
    def publish_tf(self, pose_msg: PoseStamped):
        """Publish TF transform from camera_frame → marker_frame."""

        t = TransformStamped()

        t.header.stamp = pose_msg.header.stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = "aruco_marker"

        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z

        t.transform.rotation = pose_msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    node = ArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()