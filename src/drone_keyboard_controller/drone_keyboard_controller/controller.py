import rclpy
from rclpy.node import Node
import keyboard as k
import time

from mavros_msgs.msg import  State
from geometry_msgs.msg import TwistStamped

from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool

from enum import Enum

state = State()
cmd = TwistStamped()



class FlightState(Enum):
    PREFLIGHT = "start_mission"
    TAKEOFF = "start_takeoff"
    MISSION = "mission_complete"
    LAND = "abort"


class Controller(Node):
    
    def __init__(self):
        self.state = FlightState.PREFLIGHT

        self.transitions = {(FlightState.PREFLIGHT,"preflight_ok"): (FlightState.TAKEOFF, self.on_mission_start),
                            (FlightState.TAKEOFF,"takeoff_complete"):(FlightState.MISSION, self.on_mission_start)
                       }


        super().__init__('controller_node')

        self.current_mode = "UNKNOWN"
        self.is_armed = False
        self.is_connected = False

        ####################### SERVICES DECLARATION #################################
        self.arm_srv = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_srv = self.create_client(SetMode, "/mavros/set_mode")


        ####################### PUBLISHERS DECLARATION ###############################
        self.subscription = self.create_subscription(
                                                    State,
                                                    'mavros/state',
                                                    self.state_callback,
                                                    10
        )

        self.vel_pub = self.create_publisher(
                                            TwistStamped,
                                            '/mavros/setpoint_velocity/cmd_vel',
                                            10
                                            )

        
        # Main control loop at 10 Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)


    def on_mission_start(self):
        return
    

    def state_callback(self,msg:State):
        # Update variables from subscriber callback
        self.current_mode = msg.mode
        self.is_armed = msg.armed
        self.is_connected = msg.connected
        if (self.state == FlightState.PREFLIGHT 
                                                and msg.mode == "OFFBOARD" 
                                                and msg.armed 
                                                and msg.connected):
            self.get_logger().info("Preflight OK, triggering transition")
            self.trigger("preflight_ok")

    def trigger(self,event:str):
        key = (self.state,event)
        if key not in self.transitions:
            self.get_logger().warn(
                f"Ignored event '{event}' in state '{self.state.name}'"
            )
            return self.state
        
        next_state, action = self.transitions[key]

        # Log the transition
        self.get_logger().info(
                        f"Transition: {self.state.name} --({event})-> {next_state.name}"
                        )

        action()
        self.state = next_state

        return self.state

    def control_loop(self):
        if self.state == FlightState.PREFLIGHT:
            #needs proper implementation
            self.arm_check()
            self.publish_preflight_setpoint()
            self.make_requests()
        if self.state == FlightState.MISSION:
            #self.publish_mission_setpoints()
            self.key_control()
            #keyboard controler
        #if self.state == FlightState.LAND:
            #wait to land
        return
    
    #make the drone circle around
    def publish_mission_setpoints(self):
        msg = TwistStamped()

        msg.twist.linear.x = 1.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = 0.2

        self.vel_pub.publish(msg)


    #this function publish a cmd vel for the drone to stay still 
    def publish_preflight_setpoint(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        # Zero velocity → hold still
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.z = 0.0  # no yaw rotation

        self.vel_pub.publish(msg)

    def make_requests(self):
        mode_req = SetMode.Request()
        mode_req.custom_mode = "OFFBOARD"
        self.mode_srv.call_async(mode_req)
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.arm_srv.call_async(arm_req)    
        return
            

    def key_control(self):
        msg = TwistStamped()
        linear_speed = 1.0      # m/s
        vertical_speed = 0.5    # m/s
        angular_speed = 0.5     # rad/s
        #x axis
        if k.is_pressed('w'):
            msg.twist.linear.x = linear_speed
        elif k.is_pressed('s'):
            msg.twist.linear.x = -linear_speed
        else:
            msg.twist.linear.x = 0

        #y axis 
        if k.is_pressed('a'):
            msg.twist.linear.y = linear_speed
        elif k.is_pressed('d'):
            msg.twist.linear.y = -linear_speed
        else:
            msg.twist.linear.y = 0

        #z axis
        if k.is_pressed('r'):
            msg.twist.linear.z = vertical_speed
        elif k.is_pressed('f'):
            msg.twist.linear.z = -vertical_speed
        else:
            msg.twist.linear.z = 0

        #r axis
        if k.is_pressed('q'):
            msg.twist.angular.z = angular_speed
        elif k.is_pressed('e'):
            msg.twist.angular.z = -angular_speed
        else:
            msg.twist.angular.z = 0

        self.vel_pub.publish(msg)
        return

    def arm_check(self):
        if not self.is_connected:
            #self.get_logger().info("Waiting for FCU connection...")
            return
        
        if self.current_mode == "OFFBOARD":
            self.get_logger().info("Drone is armed and in OFFBOARD mode")
            #do a circle
            
        else:
            self.get_logger().info(
                                    f"Mode: {self.current_mode}, Armed: {self.is_armed}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()