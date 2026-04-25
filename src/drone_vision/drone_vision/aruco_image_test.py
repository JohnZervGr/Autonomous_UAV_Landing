#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ArucoImageTest(Node):
    def __init__(self):
        super().__init__('aruco_image_test')

        self.declare_parameter('image_topic', '/gz_camera/image_raw')
        self.image_topic = self.get_parameter('image_topic').value

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Subscribed to: {self.image_topic}')
        self.get_logger().info('Using ArUco dictionary: DICT_4X4_50')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.parameters
        )

        if ids is not None:
            self.get_logger().info(f'Detected marker IDs: {ids.flatten().tolist()}')
        else:
            self.get_logger().info('No marker detected')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoImageTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()