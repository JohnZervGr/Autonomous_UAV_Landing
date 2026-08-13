import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Vector3, TwistStamped

class ProportionalNav(Node):
    def __init__(self):
        super().__init__('proportional_nav_node')

        self.declare_parameter("Kp", 0.2)
        self.Kp = self.get_parameter("Kp").value

        self.get_logger().info(f"Proportional navigation controller initialized with Kp {self.Kp}")

        self.detected = False
        self.det_msg = Bool()

        self.error = Vector3()
        self.error.x = 0.0
        self.error.y = 0.0

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/guidance/error',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.error.x = msg.pose.position.x
        self.error.y = msg.pose.position.y
        self.error.z = msg.pose.position.z

        #extract LOS angle
        mag = np.linalg.norm([self.error.x, self.error.y, self.error.z])
        angle_x = np.arcos(self.error.x/mag)
        angle_y = np.arcos(self.error.y/mag)

        self.get_logger().info(f"LOS angles: {np.degrees(angle_x)}, {np.degrees(angle_y)}")

        
        #produce the correction command from the PN eqasions


        return


def main(args=None):
    rclpy.init(args=args)
    proportional_nav = ProportionalNav()
    rclpy.spin(proportional_nav)
    proportional_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()