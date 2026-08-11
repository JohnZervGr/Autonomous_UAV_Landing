import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np

class Guidance(Node):
    def __init__(self):
        super().__init__("guidance_node")

        self.point_roll = np.pi/2 #90 degrees up
        self.point_pitch = 0.0
        self.point_yaw = 0.0
        self.target_point = np.array([0,0,10])

        self.create_subscription(PoseStamped,
                                "/aruco/pose", 
                                self.pose_callback, 
                                10)

        self.error_pub = self.create_publisher(PoseStamped,
                                          'guidance/error',
                                          10)


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
        error_msg = PoseStamped()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.pose.position.x = error[1]
        error_msg.pose.position.y = error[0]
        error_msg.pose.position.z = error[2]

        error_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) #placeholder

        self.error_pub.publish(error_msg)

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
