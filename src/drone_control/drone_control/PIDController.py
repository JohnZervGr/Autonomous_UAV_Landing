#ros imports
from math import tanh

import rclpy
from rclpy.node import Node

#ros msg imports
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped,Vector3,TwistStamped

from std_srvs.srv import SetBool




class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.last_detection = self.get_clock().now()

        self.declare_parameter("timeout",0.5)
        self.declare_parameter("alpha",0.2)
        self.declare_parameter
        
        #parametrise this
        self.detection_timeout = self.get_parameter("timeout").value
        self.alpha = self.get_parameter("alpha").value

        # PID gains
        self.Ke = Vector3(
            x=self.declare_parameter("Ke.x", 0.2).value,
            y=self.declare_parameter("Ke.y", 0.2).value,
            z=self.declare_parameter("Ke.z", 0.2).value,
        )

        self.Ki = Vector3(
            x=self.declare_parameter("Ki.x", 0.0).value,
            y=self.declare_parameter("Ki.y", 0.0).value,
            z=self.declare_parameter("Ki.z", 0.0).value,
        )

        self.Kd = Vector3(
            x=self.declare_parameter("Kd.x", 0.0).value,
            y=self.declare_parameter("Kd.y", 0.0).value,
            z=self.declare_parameter("Kd.z", 0.0).value,
        )

        self.limit = Vector3(
            x=self.declare_parameter("limit.x", 3.0).value,
            y=self.declare_parameter("limit.y", 3.0).value,
            z=self.declare_parameter("limit.z", 1.2).value,
        )


        self.create_service(SetBool, 'decent', self.decent_cb)
        self.get_logger().info(f"controller inisialised with P {self.Ke.x}")

        self.decent = False
        self.detected = False
        self.det_msg = Bool()
        self.k_v = 0.5 #vertical velocity gain during decent phase, this will be parametrised later
        self.k_h = 0.5 #horizontal velocity gain during decent phase, this will be parametrised later

        self.error = Vector3()
        self.error.x = 0.0
        self.error.y = 0.0
        self.error.z = 0.0

        # this will change to be given by another node for approach
        self.offset = Vector3()
        self.offset.x = 0.0
        self.offset.y = 0.0
        self.offset.z = 4.2


        self.d_f = Vector3()
        self.d_f.x = 0.0
        self.d_f.y = 0.0
        self.d_f.z = 0.0

        # integral part 
        self.i = Vector3()
        self.i.x = 0
        self.i.y = 0
        self.i.z = 0

        self.max_i = Vector3(x=0.5 , y=0.5 , z=0.5)


        self.correction = TwistStamped()
        self.correction.twist.linear.x = 0.0
        self.correction.twist.linear.y = 0.0
        self.correction.twist.linear.z = 0.0


        self.create_subscription(PoseStamped,
                                 "/guidance/error",
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
    

    def decent_cb(self,request,response):
        self.decent = request.data
        self.get_logger().info(f"decent mode set to {self.decent}")
        response.success = True
        return response
    
    def read_pose_cb(self,msg:PoseStamped):

        now = self.get_clock().now()
        dt = max((now - self.last_detection).nanoseconds * 1e-9, 1e-6)

        #calculate error part
        e = Vector3()
        e.x = msg.pose.position.x - self.offset.x
        e.y = -msg.pose.position.y + self.offset.y
        e.z = msg.pose.position.z - self.offset.z
        #calculate derivative part

        d = Vector3()
        d.x = (e.x - self.error.x) / dt 
        d.y = (e.y - self.error.y) / dt
        d.z = (e.z - self.error.z) / dt

        # low pass filter over derivative to prevent kicks
        self.d_f.x = (self.alpha * self.d_f.x) + (1- self.alpha)*d.x
        self.d_f.y = (self.alpha * self.d_f.y) + (1- self.alpha)*d.y
        self.d_f.z = (self.alpha * self.d_f.z) + (1- self.alpha)*d.z

        #calculate intergral part
        self.i.x += self.clamp(e.x * dt, self.max_i.x)
        self.i.y += self.clamp(e.y * dt, self.max_i.y)
        self.i.z += self.clamp(e.z * dt, self.max_i.z)

        self.correction.twist.linear.x = self.clamp(self.Ke.x * e.x + 
                                                    (self.Kd.x) * self.d_f.x + 
                                                    self.Ki.x * self.i.x
                                                    ,self.limit.x)
        self.correction.twist.linear.y = self.clamp(self.Ke.y * e.y + 
                                                    (self.Kd.y) * self.d_f.y + 
                                                    self.Ki.y * self.i.y
                                                    ,self.limit.y)
        self.correction.twist.linear.z = self.clamp(self.Ke.z * e.z + 
                                                    (self.Kd.z) * self.d_f.z + 
                                                    self.Ki.z * self.i.z
                                                    ,self.limit.z)

        #when in decent phase,overwrite z correction 
        #z speed is reduced the closer the drone is to the ground
        #z speed is reduced the further away the drone is from the center of the marker
        if self.decent:
            
            h_dist = self.error.x**2 + self.error.y**2
            self.correction.twist.linear.z = (self.limit.z *                            #max allowable speed
                                              tanh(self.k_v * (self.error.z - 2.2)) *  #vertical clamping
                                              tanh(self.k_h * (1-h_dist)))                  #horizontal clamping
            self.get_logger().info(f"decent mode active v speed {self.correction.twist.linear.z} h_dist {h_dist} z error {self.error.z}")
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
        
        self.det_msg.data = not stale

        #format messages
        cor_msg = self.correction
        cor_msg.header.stamp = self.get_clock().now().to_msg()

        #publish
        self.detection_status_pub.publish(self.det_msg)
        self.correction_pub.publish(cor_msg)
        return
    

def main(args=None):
    rclpy.init(args=args)
    pidController = PIDController()
    rclpy.spin(pidController)
    pidController.destroy_node()
    rclpy.shutdown()