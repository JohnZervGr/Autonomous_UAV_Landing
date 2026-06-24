#ros imports
import rclpy
from rclpy.node import Node
#mavros msg imports
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
#ros msg imports
from std_msgs.msg import Bool

from enum import Enum

# Define flight states for FSM
class FlightState(Enum):
    UNKNOWN = "unknown"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    MISSION = "mission"
    LANDING = "landing"

class Mission_Controller(Node):
    def __init__(self):
        super().__init__('mission_controller')

        #fsm variables
        self.state_handlers = {
            FlightState.UNKNOWN: self.run_unknown,
            FlightState.PREFLIGHT: self.run_preflight,
            FlightState.TAKEOFF: self.run_takeoff,
            FlightState.MISSION: self.run_mission,
            #FlightState.LANDING: self.run_landing,
        }

        self.transitions = {
            (FlightState.UNKNOWN, "connection_established"): (FlightState.PREFLIGHT, self.on_preflight_start),
            (FlightState.PREFLIGHT, "ready_for_takeoff"): (FlightState.TAKEOFF, self.on_ready_for_takeoff),
            (FlightState.TAKEOFF, "takeoff_complete"): (FlightState.MISSION, self.on_mission_start),
            (FlightState.LANDING, "marker_lost"): (FlightState.LANDING, self.on_mission_complete),
        }
        self.fsm_state = FlightState.UNKNOWN
        self.get_logger().info("fsm initialised")

        #preflight phase 
        self.setpoint_counter = 0
        self.current_state = State()
        self.arm_pending = False
        self.mode_pending = False

        #takeoff phase
        self.takeoff_position_reached_counter = 0
        self.current_pos = PoseStamped()
        self.current_extended_state = ExtendedState()
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 3.0)
        self.declare_parameter('pos_tolerance', 0.20)
        
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_z = self.get_parameter('target_z').value
        self.pos_tolerance = self.get_parameter('pos_tolerance').value

        #subscribers
        self.create_subscription(State, 
                                 '/mavros/state', 
                                 self.state_cb, 
                                 10)
        
        self.create_subscription(ExtendedState,
                                 '/mavros/extended_state',
                                 self.extended_state_cb,
                                 10)
        self.create_subscription(PoseStamped,
                                 '/mavros/local_position/pose',
                                 self.pose_cb,
                                 10)
        self.get_logger().info("topic subscriptions initialized")

        #publisers
        self.pose_pub = self.create_publisher(
                                 PoseStamped,
                                 '/mavros/setpoint_position/local',
                                 10
        )

        self.vel_pub = self.create_publisher(
                                 TwistStamped,
                                 '/mavros/setpoint_velocity/cmd_vel',
                                 10
        )
        self.get_logger().info("topic publishers initialized")


        #services
        self.arm_srv = self.create_client(CommandBool, 
                                          "/mavros/cmd/arming")
        self.mode_srv = self.create_client(SetMode,
                                           "/mavros/set_mode")
        self.get_logger().info("services initialized")


        #control loop timer
        self.create_timer(0.1, self.controller_loop)
        self.get_logger().info("controll timer started")
        self.get_logger().info("node initialiation complete")

    '''
    ##########################################################
                    SERVICES FUNCTIONS
    ##########################################################
    '''

    def arm_drone(self):
        if self.current_state.armed: return
        if self.arm_pending : return

        serv_req = CommandBool.Request()
        serv_req.value = True

        future = self.arm_srv.call_async(serv_req)
        self.arm_pending = True

        future.add_done_callback(self.arm_cb)
        return
    
    
    def set_mode_offboard(self):
        if self.current_state.mode == "OFFBOARD": return
        if self.mode_pending:return

        mode_req = SetMode.Request()
        mode_req.custom_mode = "OFFBOARD"
        future = self.mode_srv.call_async(mode_req)
        self.mode_pending = True
        
        future.add_done_callback(self.offbrd_cb)
        return
    '''
    ##########################################################
                    PUBLISHER FUNCTIONS
    ##########################################################
    '''

    def publish_takeoff_setpoint(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z
        self.pose_pub.publish(msg)
        return
    
    def publish_velocity_setpoint(self,x = 0.0,y = 0.0,z = 0.0):
        msg = TwistStamped()
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = z
        msg.header.stamp = self.get_clock().now().to_msg()
        self.vel_pub.publish(msg)
        return
    
    '''
    ##########################################################
                    CALLBACK FUNCTIONS
    ##########################################################
    '''

    def state_cb(self, msg):
        self.current_state = msg
        return
    
    def extended_state_cb(self , msg):
        self.current_extended_state = msg
        return
    
    def pose_cb(self, msg):
        self.current_pos = msg
        return
    
    def offbrd_cb(self,future):
        self.mode_pending = False
        if not self.current_state.mode == "OFFBOARD" : return
        self.get_logger().info("mode set: OFFBOARD")
        return
    
    def arm_cb(self,future):
        self.arm_pending = False
        if not self.current_state.armed: return
        self.get_logger().info("arming completed")
        return
    
    '''
    ##########################################################
                    FSM TRANSITION FUNCTIONS
    ##########################################################
    '''

    def transition(self, event):
        key = (self.fsm_state, event)
        if key in self.transitions:
            new_state, action = self.transitions[key]
            self.get_logger().info(f"Transitioning from {self.fsm_state} to {new_state} on event {event}")
            self.fsm_state = new_state
            if action:
                action()
        else:
            self.get_logger().warn(f"No transition defined for state {self.fsm_state} on event {event}")
    

    def on_ready_for_takeoff(self):
        self.takeoff_position_reached_counter = 0
        return
    
    def on_mission_start(self):
        return  
    
    def on_mission_complete(self):
        return
    
    def on_preflight_start(self):
        self.setpoint_counter = 0
        return
   
    '''
    ##########################################################
                    CONTROL FUNCTIONS
    ##########################################################
    '''
    def controller_loop(self):
        #publish
        self.state_handlers[self.fsm_state]()
        return
    
    def run_unknown(self):
        self.publish_velocity_setpoint()
        self.setpoint_counter += 1

        if self.setpoint_counter < 20: return
        if not self.current_state.connected: return
        self.transition("connection_established")

    def run_preflight(self):
        self.publish_velocity_setpoint()
        self.setpoint_counter+=1

        #wait until offboard conditions are satisfied
        if self.setpoint_counter < 20 :return

        #make offboard request if there is need
        if self.current_state.mode != "OFFBOARD":
            self.set_mode_offboard()
            return
        
        #arm drone if not already armed
        if not self.current_state.armed:
            self.arm_drone()
            return

        #check transition parameters
        if not (self.current_state.connected and self.current_state.armed and self.current_state.mode == "OFFBOARD"): return
        
        self.transition("ready_for_takeoff")

    def run_takeoff(self):
        self.publish_takeoff_setpoint()

        #self.check_position_reached()
        if abs(self.target_x - self.current_pos.pose.position.x) < self.pos_tolerance and \
           abs(self.target_y - self.current_pos.pose.position.y) < self.pos_tolerance and \
           abs(self.target_z - self.current_pos.pose.position.z) < self.pos_tolerance:
                self.takeoff_position_reached_counter += 1
        else:
            self.takeoff_position_reached_counter = 0
            
        
        if self.takeoff_position_reached_counter >= 5:
            self.transition("takeoff_complete")

    def run_mission(self):
        self.publish_takeoff_setpoint() #placeholder pid controllers will be called here
    
    

def main(args=None):
    rclpy.init(args=args)
    mission_controller = Mission_Controller()
    rclpy.spin(mission_controller)
    mission_controller.destroy_node()
    rclpy.shutdown()