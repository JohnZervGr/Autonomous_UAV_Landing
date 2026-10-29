import rclpy
import numpy as np

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


class Kallman(Node):
    def __init__(self):
        super().__init__('kallman_node')

        self.declare_parameter("alpha", 0.6)
        self.alpha = self.get_parameter("alpha").value

        self.get_logger().info(f"Kallman node initialized with alpha: {self.alpha}")

        #subscribe to  imu
        self.imu_sub = self.create_subscription(Imu,
                                                "/mavros/imu/data",
                                                self.imu_callback,
                                                qos_profile_sensor_data)
        #subscribe to error
        self.marker_sub = self.create_subscription(PoseStamped,
                                                   "/guidance/error",
                                                   self.marker_callback,
                                                   10)



    def imu_callback(self, msg):
        # Process IMU data here
        #predict()
        #self.get_logger().info(f"Received IMU data: {msg}")
        return

    def marker_callback(self, msg):
        # Process marker data here
        #update()   
        #self.get_logger().info(f"Received marker data: {msg}")
        return


    def init_filter(self, F, B, H, Q, R, x0, P0):
        """
        Initialize the Kalman filter parameters.
        :param F: State transition matrix.
        :param B: Control input matrix.
        :param H: Observation matrix.
        :param Q: Process noise covariance.
        :param R: Measurement noise covariance.
        :param x0: Initial state estimate.
        :param P0: Initial estimate covariance.
        """
        self.F = F
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.x = x0  # State estimate
        self.P = P0  # Estimate covariance
        return

    def predict(self, u, dt):
        """
        Predict the next state and estimate covariance.
        :param u: Control input.
        :param dt: Time step.
        """
        F = np.array([1, dt],[0, 1])

        self.x = np.dot(F, self.x) + np.dot(self.B, u)
        self.P = np.dot(F, np.dot(self.P, F.T)) + self.Q
        return self.x, self.P

    def update(self, z):
        """
        Update the state estimate and estimate covariance with a new measurement.
        :param z: The new measurement (numpy array).
        """
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R   #innovation cov
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # kallman gain
        y = z - np.dot(self.H, self.x)                          #inovation update
        self.x = self.x + np.dot(K, y)                          #estimate update
        I = np.eye(self.P.shape[0]) 
        self.P = np.dot(I - np.dot(K, self.H), self.P)          #cov matrix
        return self.x
    
    #set the message timing
    def set_message_timing(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.get_logger().info(f"Message timing set at: {self.last_msg_time}")


    #set the kallman filter 
    def kallman_filter(self, measurement, estimate):
        """
        Apply the Kalman filter update step.
        :param measurement: The new measurement (numpy array).
        :param estimate: The current estimate (numpy array).
        :return: Updated estimate after applying the Kalman filter.
        """
        # Update the estimate with the new measurement
        updated_estimate = self.alpha * estimate + (1 - self.alpha) * measurement
        return updated_estimate


def main(args=None):
    rclpy.init(args=args)
    node = Kallman()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




