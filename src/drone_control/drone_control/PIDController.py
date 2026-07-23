#ros imports
import rclpy
from rclpy.node import Node

#ros msg imports
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Vector3Stamped,Vector3,TwistStamped




class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.last_detection = self.get_clock().now()
        #parametrise this
        self.detection_timeout = 0.5
        #parametrrise this
        # low pass filter value
        self.alpha = 0.2

        self.error = Vector3()
        self.error.x = 0.0
        self.error.y = 0.0
        self.error.z = 0.0

        # this will change to be given by another node for approach
        self.target = Vector3()
        self.target.x = 0.0
        self.target.y = 0.0
        self.target.z = 2.0

        #(parametrise these)
        # PID gain values
        self.Ke = Vector3()
        self.Ke.x = 0.5
        self.Ke.y=  0.5
        self.Ke.z = 0.5

        self.Kd = Vector3()
        self.Kd.x = 0.5
        self.Kd.y=  0.5
        self.Kd.z = 0.5

        self.d_f = Vector3()
        self.d_f.x = 0.0
        self.d_f.y = 0.0
        self.d_f.z = 0.0

        self.Ki = Vector3()
        self.Ki.x = 0.0
        self.Ki.y=  0.0
        self.Ki.z = 0.0
        # integral part 
        self.i = Vector3()
        self.i.x = 0
        self.i.y = 0
        self.i.z = 0

        #(parametrise these)
        #clamping limits
        self.limit = Vector3()
        self.limit.x = 3.0
        self.limit.y = 3.0
        self.limit.z = 1.2

        self.correction = TwistStamped()
        self.correction.twist.linear.x = 0.0
        self.correction.twist.linear.y = 0.0
        self.correction.twist.linear.z = 0.0


        self.create_subscription(PoseStamped,
                                 "/aruco/pose",
                                 self.read_pose_cb,
                                 10
                                )
        
        self.correction_pub = self.create_publisher(TwistStamped,
                                                    'controller/cmd_vel',
                                                    10)
        
        self.detection_status_pub = self.create_publisher(Bool,
                                                          'controller/status',
                                                          10)
                
        self.create_timer(0.01,self.publish_correction) #set the publication timer to 100hz (max)
        self.get_logger().info("Node initialised")

        return
    
    
    def read_pose_cb(self,msg:PoseStamped):

        now = self.get_clock().now()
        dt = max((now - self.last_detection).nanoseconds * 1e-9, 1e-6)

        #calculate error part
        e = Vector3()
        e.x = self.target.x -  msg.pose.position.x
        e.y = self.target.y -  msg.pose.position.y
        e.z = self.target.z -  msg.pose.position.z
        #calculate derivative part

        d = Vector3()
        d.x = (e.x - self.error.x) / dt 
        #filtered_derivative = (alpha * filtered_derivative) + (1 - alpha) * derivative;
        d.y = (e.y - self.error.y) / dt
        d.z = (e.z - self.error.z) / dt

        # low pass filter over derivative to prevent kicks
        self.d_f.x = (self.alpha * self.d_f.x) + (1- self.alpha)*d.x
        self.d_f.y = (self.alpha * self.d_f.y) + (1- self.alpha)*d.y
        self.d_f.z = (self.alpha * self.d_f.z) + (1- self.alpha)*d.z

        #calculate intergral part
        self.i.x += e.x * dt
        self.i.y += e.y * dt
        self.i.z += e.z * dt

        self.correction.twist.linear.x = self.clamp(self.Ke.x * e.x + 
                                                    self.Kd.x * self.d_f.x + 
                                                    self.Ki.x * self.i.x
                                                    ,self.limit.x)
        self.correction.twist.linear.y = self.clamp(self.Ke.y * e.y + 
                                                    self.Kd.y * self.d_f.y + 
                                                    self.Ki.y * self.i.y
                                                    ,self.limit.y)
        self.correction.twist.linear.z = self.clamp(self.Ke.z * e.z + 
                                                    self.Kd.z * self.d_f.z + 
                                                    self.Ki.z * self.i.z
                                                    ,self.limit.z)
        # calculate pid correction

        self.error.x = e.x
        self.error.y = e.y
        self.error.z = e.z
        self.last_detection = now
        return
    
    def clamp(self, val, max_val):
        return max(min(val, max_val), -max_val)
    
    def publish_correction(self):
        #make sure its not a stale detection
        det_msg = Bool()
        #cor_msg = TwistStamped()
        stale = self.get_clock().now() - self.last_detection > rclpy.duration.Duration(seconds=self.detection_timeout)

        if stale:
            #reset intergral part
            self.i.x = 0.0
            self.i.y = 0.0
            self.i.z = 0.0
            #reset derivative part
            self.d_f.x = 0.0
            self.d_f.y = 0.0 
            self.d_f.z = 0.0
        
        det_msg.data = not stale

        #format messages
        cor_msg = self.correction
        cor_msg.header.stamp = self.get_clock().now().to_msg()

        #publish
        self.detection_status_pub.publish(det_msg)
        self.correction_pub.publish(cor_msg)
        return
    

def main(args=None):
    rclpy.init(args=args)
    pidController = PIDController()
    rclpy.spin(pidController)
    pidController.destroy_node()
    rclpy.shutdown()