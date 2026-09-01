#ros imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
#mavros msg imports
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
#ros msg imports
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from rclpy.qos import qos_profile_sensor_data

from enum import Enum

# Define flight states for FSM
class FlightState(Enum):
    UNKNOWN = "unknown"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    MISSION = "mission"
    TRACK = "tracking"
    DESCENT = "descent"
    TOUCHDOWN = "touchdown"

class Mission_Controller(Node):
    def __init__(self):
        super().__init__('mission_controller')

        #fsm variables
        self.state_handlers = {
            FlightState.UNKNOWN: self.run_unknown,
            FlightState.PREFLIGHT: self.run_preflight,
            FlightState.TAKEOFF: self.run_takeoff,
            FlightState.MISSION: self.run_mission,
            FlightState.TRACK: self.run_tracking,
            FlightState.DESCENT: self.run_descent,
            FlightState.TOUCHDOWN: self.run_touchdown
        }

        self.transitions = {
            (FlightState.UNKNOWN, "connection_established"): (FlightState.PREFLIGHT, self.on_preflight_start),
            (FlightState.PREFLIGHT, "ready_for_takeoff"): (FlightState.TAKEOFF, self.on__takeoff_start),
            (FlightState.TAKEOFF, "takeoff_complete"): (FlightState.MISSION, self.on_mission_start),
            (FlightState.MISSION, "marker_found"): (FlightState.TRACK, self.on_tracking_start),
            (FlightState.TRACK, "marker_lost"): (FlightState.MISSION, self.on_marker_lost),
            (FlightState.TRACK, "controller_lost"): (FlightState.MISSION, self.on_marker_lost),     #on function might change later
            (FlightState.TRACK, "approach_stable"):(FlightState.DESCENT, self.on_descending_start), 
            (FlightState.DESCENT, "marker_lost"): (FlightState.MISSION, self.on_marker_lost),
            (FlightState.DESCENT, "controller_lost"): (FlightState.MISSION, self.on_marker_lost),
            (FlightState.DESCENT, "ready_for_landing"): (FlightState.TOUCHDOWN, self.on_touchdown_start),
            (FlightState.TOUCHDOWN, "restart_mission"):(FlightState.PREFLIGHT)                        #usless for now
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
        self.declare_parameter('pos_tolerance', 1.80)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_z = self.get_parameter('target_z').value

        self.pos_tolerance = self.get_parameter('pos_tolerance').value


        #mission phase

        #tracking phase
        self.last_correction = TwistStamped()
        self.detection = False
        self.correction_timer = self.create_timer(1/100,self.correction_fwrd_cb,autostart=False)
        self.pos_error = PoseStamped()

        #descent phase
        self.descenting = False
        self.dec_mode_pending = False
        self.z_offset = 5.0

        #touchdown phase
        self.land_req_pending = False
        self.landing = False


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
                                 qos_profile_sensor_data)
        self.create_subscription(Bool,
                                 '/controller/status',
                                 self.detection_cb,
                                 10)
        self.create_subscription(TwistStamped,
                                 '/controller/cmd_vel',
                                 self.read_controller_correction_cb,
                                 10)
        self.create_subscription(PoseStamped,
                                "/guidance/error",
                                self.read_error_cb,
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
        self.descent_srv = self.create_client(SetBool,
                                             "/decent")
        self.land_srv = self.create_client(CommandTOL,
                                           "/mavros/cmd/land")
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

    def set_descent(self,data:Bool):
        #entry checks
        if self.descenting == data:return
        if self.dec_mode_pending:return
        #create request
        dec_req = SetBool.Request()
        dec_req.data = data
        #send req
        future = self.descent_srv.call_async(dec_req)
        self.dec_mode_pending = True
        #set cb
        future.add_done_callback(self.descent_cb)
        return

    def set_land(self):
        if self.land_req_pending:return
        if self.landing :return
        self.get_logger().info("landing requested")
        land_req = CommandTOL.Request()
        future = self.land_srv.call_async(land_req)
        self.land_req_pending = True
        future.add_done_callback(self.land_cb)
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

    def correction_fwrd_cb(self):
        self.vel_pub.publish(self.last_correction)
        return
    
    '''
    ##########################################################
                    CALLBACK FUNCTIONS
    ##########################################################
    '''

    def read_error_cb(self, msg:PoseStamped):
        self.pos_error = msg
        return

    def state_cb(self, msg:State):
        self.current_state = msg
        return
    
    def extended_state_cb(self , msg:ExtendedState):
        self.current_extended_state = msg
        return
    
    def pose_cb(self, msg:PoseStamped):
        self.current_pos = msg
        #self.get_logger().info(f"position recieved {self.current_pos}")
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

    def descent_cb(self,future):
        self.dec_mode_pending = False
        if not future.result():return
        self.descenting = True
        return

    def land_cb(self,future):
        self.land_req_pending = False
        if not future.result():return
        self.landing = True
        return
    
    def read_controller_correction_cb(self,msg:TwistStamped):
        self.last_correction = msg

    def detection_cb(self,msg:Bool):
        self.detection = msg.data
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
            self.get_logger().warn(f"No transition defined for state {self.fsm_state} from on event {event}")
    
    def on_marker_lost(self):
        self.correction_timer.cancel()
        self.descenting = False
        self.dec_mode_pending = False
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

    '''
    eshtablise connectiopn with FC 
    begin publish offboard signal (heartbeat)
    '''
    def run_unknown(self):
        self.publish_velocity_setpoint()
        self.setpoint_counter += 1

        if self.setpoint_counter < 20: return
        if not self.current_state.connected: return
        self.transition("connection_established")



    def on_preflight_start(self):
        self.setpoint_counter = 0
        return
    '''
    ensure connection with flight controller
    set flight mode to offboard
    arm drone
    '''
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


    def on__takeoff_start(self):
        self.takeoff_position_reached_counter = 0
        return

    '''
    takeoff verticaly
    '''
    def run_takeoff(self):
        self.publish_takeoff_setpoint()

        #self.check_position_reached()
        if abs(self.target_x - self.current_pos.pose.position.x) < self.pos_tolerance and \
           abs(self.target_y - self.current_pos.pose.position.y) < self.pos_tolerance and \
           abs(self.target_z - self.current_pos.pose.position.z) < self.pos_tolerance:
                self.takeoff_position_reached_counter += 1
        else:
            self.takeoff_position_reached_counter = 0
            
        #self.get_logger().info(f"position {abs(self.current_pos.pose.position.z)} reached for {self.takeoff_position_reached_counter} ticks")
        if self.takeoff_position_reached_counter >= 5:
            self.transition("takeoff_complete")

    
    def on_mission_start(self):
        return

    '''
    go to a predecided position where the marker is visible
    '''
    def run_mission(self):
        self.publish_takeoff_setpoint() #placeholder will be set to a random position close to the marker
        #if marker is detected transition to tracking
        stale = self.get_clock().now() - Time.from_msg(self.last_correction.header.stamp) > rclpy.duration.Duration(seconds=0.1)

        if self.detection and not stale:
            self.transition("marker_found")
        return

    def on_tracking_start(self):
        self.correction_timer.reset()
        self.descenting = False
        self.dec_mode_pending = False
        self.tracking_position_reached_counter = 0
        return    
    '''
    align drone above the marker
    when the correction remain constant for more that a second go to descent
    if the marker is lost swap to mission
    '''
    def run_tracking(self):

        stale = self.get_clock().now() - Time.from_msg(self.last_correction.header.stamp) > rclpy.duration.Duration(seconds=0.1)
        #marker is lost, transition away from tracking
        if not self.detection:
            self.transition("marker_lost")

        #look if controller msgs are stale
        if stale:
            self.transition("marker_lost") 


        #self.check_position_reached()
        if abs(self.target_x - self.pos_error.pose.position.x) < self.pos_tolerance and \
           abs(self.target_y - self.pos_error.pose.position.y) < self.pos_tolerance and \
           abs(self.target_z - self.pos_error.pose.position.z) < self.pos_tolerance:
                self.tracking_position_reached_counter += 1
        else:
            self.tracking_position_reached_counter = 0   

        if self.tracking_position_reached_counter > 5:
            self.transition("approach_stable")

        return


    def on_descending_start(self):
        self.descenting = False
        self.dec_mode_pending = False
        return
    
    def run_descent(self):
        #final approach to marker and transition to towtchdown
        stale = self.get_clock().now() - Time.from_msg(self.last_correction.header.stamp) > rclpy.duration.Duration(seconds=0.1)
        #marker is lost, transition away from tracking
        if not self.detection:
            self.transition("marker_lost")

        #look if controller msgs are stale
        if stale:
            self.transition("marker_lost")    

        #keep checking for stable
        #ensure stable apporach
        self.cor_threshold = 0.3 #will move to init will be parametrised

        x_err = abs(self.pos_error.pose.position.x)
        y_err = abs(self.pos_error.pose.position.y)
        z_err = (self.pos_error.pose.position.z)
        stable = (x_err < self.cor_threshold and y_err < self.cor_threshold)

        #make descent request if needed
        if stable and not self.descenting:
            self.set_descent(True)

        if not stable and self.descenting:
            self.set_descent(False)

        #exit to twichdown checks
        #when close egnough to the pad transition to towcdown
        if stable and self.z_offset - z_err < 0.3:
            self.transition("ready_for_landing")

        return

    def on_touchdown_start(self):
        
        return

    def run_touchdown(self):
        #check if relative position is good
        self.set_land()
        #request disarm and land

        #wait for user input to restart the mission
        return
    
    

def main(args=None):
    rclpy.init(args=args)
    mission_controller = Mission_Controller()
    rclpy.spin(mission_controller)
    mission_controller.destroy_node()
    rclpy.shutdown()