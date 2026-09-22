import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

from landing_interfaces.srv import SetPose

import numpy as np

class Guidance(Node):
    def __init__(self):
        super().__init__("guidance_node")

        self.target_point = np.array([0,0,5])
        self.target_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.alpha = 0.6

        self.create_subscription(PoseStamped,
                                "/aruco/pose", 
                                self.pose_callback, 
                                10)

        self.error_pub = self.create_publisher(PoseStamped,
                                          'guidance/error',
                                          10)

        self.tgt_srv = self.create_service(SetPose,
                                           'guidance/set_target',
                                           self.set_target_callback)

        self.error_msg = PoseStamped()
        self.error_msg.header.stamp = self.get_clock().now().to_msg()
        self.error_msg.pose.position.x = 0.0
        self.error_msg.pose.position.y = 0.0
        self.error_msg.pose.position.z = 0.0

        self.error_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) 
        self.get_logger().info("Guidance node initialized")

    def set_target_callback(self, request, response):
        self.target_point = np.array([request.target_pose.position.y,
                                      request.target_pose.position.x,
                                      request.target_pose.position.z])
        self.target_orientation = request.target_pose.orientation
        self.get_logger().info(f"Target point set to: {self.target_point}")
        response.success = True
        return response

    
    def rotate_vector(self,v: np.ndarray , q: Quaternion) -> np.ndarray:
        """
        Rotate a 3D vector v by a unit quaternion q, using the dot-product
        closed form of q * v * q_conjugate:
    
            v' = v*(w^2 - u.u) + 2*(u.v)*u + 2*w*(u x v)

        ROS2 quaternion convention: (x, y, z, w).
        """
        u = np.array([q.x, q.y, q.z])   # vector part
        w = q.w                          # scalar part
    
        return v * (w * w - np.dot(u, u)) + 2.0 * np.dot(u, v) * u + 2.0 * w * np.cross(u, v)

        
    def pose_callback(self, msg: PoseStamped):
        v_origin = np.array([msg.pose.position.y,
                              msg.pose.position.x,
                              msg.pose.position.z])

        q = msg.pose.orientation

        q_inverse = Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)
        t_rotated = self.rotate_vector(v_origin, q_inverse)
        error = self.target_point + t_rotated

        #self.get_logger().info(f"error: {error}")
        #(self.alpha * self.d_f.x) + (1- self.alpha)*d.x
        self.error_msg.header.stamp = self.get_clock().now().to_msg()
        self.error_msg.pose.position.x = (self.alpha * self.error_msg.pose.position.y) + (1 - self.alpha) * error[1]
        self.error_msg.pose.position.y = (self.alpha * self.error_msg.pose.position.x) + (1 - self.alpha) * error[0]
        self.error_msg.pose.position.z = (self.alpha * self.error_msg.pose.position.z) + (1 - self.alpha) * error[2]

        self.error_msg.pose.orientation = q_inverse #placeholder

        self.error_pub.publish(self.error_msg)

        return





def main(args=None):
    rclpy.init(args=args)
    node = Guidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
