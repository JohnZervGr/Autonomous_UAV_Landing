#ros imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
#mavros msg imports
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point
#ros msg imports
from std_srvs.srv import SetBool
from std_msgs.msg import Bool

from landing_interfaces.srv import WaypointRequest, Landing, SetPose
from rclpy.duration import Duration

from rclpy.qos import qos_profile_sensor_data

from enum import Enum

# Define flight states for FSM
class FlightState(Enum):
    UNKNOWN = "unknown"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    MISSION = "mission"
    SEARCH = "search"
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
            FlightState.SEARCH: self.run_search,
            FlightState.TRACK: self.run_tracking,
            FlightState.DESCENT: self.run_descent,
            FlightState.TOUCHDOWN: self.run_touchdown
        }

        self.transitions = {
            (FlightState.UNKNOWN, "connection_established"): (FlightState.PREFLIGHT, self.on_preflight_start),
            (FlightState.PREFLIGHT, "ready_for_takeoff"): (FlightState.TAKEOFF, self.on__takeoff_start),
            (FlightState.TAKEOFF, "takeoff_complete"): (FlightState.MISSION, self.on_mission_start),
            #(FlightState.MISSION, "marker_found"): (FlightState.TRACK, self.on_tracking_start),
            (FlightState.MISSION, "mission_complete"): (FlightState.SEARCH, self.on_search_start),
            (FlightState.SEARCH, "marker_detected"): (FlightState.TRACK, self.on_tracking_start),            
            (FlightState.TRACK, "marker_lost"): (FlightState.SEARCH, self.on_marker_lost),
            #(FlightState.TRACK, "controller_lost"): (FlightState.MISSION, self.on_marker_lost),     #on function might change later
            (FlightState.TRACK, 'tracking_stable'):(FlightState.TRACK, self.on_tracking_start),      # cet closer
            (FlightState.TRACK, "approach_stable"):(FlightState.DESCENT, self.on_descending_start), 
            (FlightState.TRACK, 'land_ready'):(FlightState.TOUCHDOWN, self.on_touchdown_start),
            (FlightState.DESCENT, "marker_lost"): (FlightState.SEARCH, self.on_marker_lost),
            (FlightState.DESCENT, "controller_lost"): (FlightState.SEARCH, self.on_marker_lost),
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

        self.mission_recieved = False

        self.tekeoff_target_pos = PoseStamped()
        self.tekeoff_target_pos_recieved = False

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

        self.takeoff_wp = PoseStamped()
        self.takeoff_wp.pose.position.x = self.get_parameter('target_x').value
        self.takeoff_wp.pose.position.y = self.get_parameter('target_y').value
        self.takeoff_wp.pose.position.z = self.get_parameter('target_z').value
        
        self.pos_tolerance = self.get_parameter('pos_tolerance').value


        #mission phase
        #initialise mission action server
        self.mission_complete = False
        self.mission_wp = PoseStamped()        

        #tracking phase
        self.last_correction = TwistStamped()
        self.detection = False
        self.pos_error = PoseStamped()
        self.tracking_offset = PoseStamped()
        self.tracking_offset.pose.position.x = 0.0
        self.tracking_offset.pose.position.y = 0.0
        self.tracking_offset.pose.position.z = 0.5

        self.du = Point()
        

        #descent phase
        self.descenting = False
        self.dec_mode_pending = False
        self.z_offset = 5.0
        self.cor_threshold = 0.3 


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
        self.waypoint_srv = self.create_client(WaypointRequest,
                                               "/mission/control")
        self.marker_srv = self.create_client(Landing,
                                              "/landing/request")
        self.land_srv = self.create_client(CommandTOL,
                                           "/mavros/cmd/land")
        self.set_controler_target_srv = self.create_client(SetPose,
                                                           "/controller/set_offset")
        self.command_srv = self.create_client(CommandLong,
                                             '/mavros/cmd/command')
        
        self.get_logger().info("services initialized")

 
        #control loop timer
        self.create_timer(0.1, self.fsm_controller)
        self.correction_timer = self.create_timer(1/100,self.flight_controller)

        self.get_logger().info("control timer started")
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

    def mission_request(self,req:int):
        if self.mission_req_pending: return
        #if self.mission_started: return
        self.get_logger().info(f"sending request: {req}")
        srv = WaypointRequest.Request()
        srv.mission_phase = req
        future = self.waypoint_srv.call_async(srv)
        self.mission_req_pending = True
        future.add_done_callback(self.mission_cb)
        return

    def send_landing_request(self):
        req = Landing.Request()
        req.request = 0
        self.la
        return

    '''
    ##########################################################
                    CALLBACK FUNCTIONS
    ##########################################################
    '''

    def read_error_cb(self, msg:PoseStamped):
        # read time difference between messages
        dt = (Time.from_msg(msg.header.stamp) - Time.from_msg(self.pos_error.header.stamp)).nanoseconds / 1e9

        # calculate relative velocity if detections are within parameters
        if (0 < dt <= 0.8):
            self.du.x = (self.pos_error.pose.position.x - msg.pose.position.x) / dt
            self.du.y = (self.pos_error.pose.position.y - msg.pose.position.y) / dt
            self.du.z = (self.pos_error.pose.position.z*(-1) - msg.pose.position.z) / dt

            vel_tol = 0.5

            # see if relative speed is within landing params
            vel_stable = (abs(self.du.x) < vel_tol
                    and abs(self.du.y) < vel_tol
                    and abs(self.du.z) < vel_tol)

            if vel_stable:
                self.du_stable = self.get_clock().now() if self.du_stable == None else self.du_stable
            else:
                self.du_stable = None
        else:
            self.du_stable = None

        # store new position
        self.pos_error = msg
        self.pos_error.pose.position.z *= -1
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

    def mission_cb(self,future:WaypointRequest.Response):
        res = future.result()
        match res.status:
            case WaypointRequest.Response.MISSION_STARTED:
                self.get_logger().info("MISSION STARTED")
                self.mission_started = True
                self.mission_complete = False
                self.get_logger().info(f"MISSION STARTED flying to waypont {self.takeoff_wp.pose.position}")

            case WaypointRequest.Response.MISSION_COMPLETE:
                self.mission_started = False
                self.mission_complete = True
                #self.mission_wp = self.takeoff_wp
                self.get_logger().info(f"MISSION COMPLETE flying to waypont {self.takeoff_wp.pose.position}")

            case WaypointRequest.Response.WAYPOINT:
                #set target
                self.mission_wp.pose = res.waypoint
                self.get_logger().info(f"WAYPOINT RECIEVED flying to waypont {self.takeoff_wp.pose.position}/{self.mission_wp.pose.position}")

            case WaypointRequest.Response.ERROR:
                self.get_logger().info("ERROR")
                #PLACEHOLDER
                #propably early transition to landing phase
                self.mission_started = False
            case _:
                self.get_logger().warn(f"unknown mission status recieved {future.status}")

        self.mission_req_pending = False
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
        #self.correction_timer.cancel()
        self.tracking_offset.pose.position.z = .5
        self.descenting = False
        self.set_descent(False)
        return

    '''
    ##########################################################
                    CONTROL FUNCTIONS
    ##########################################################
    '''

    def flight_controller(self):
        now = self.get_clock().now().to_msg()
        if self.fsm_state == FlightState.TAKEOFF:
            self.takeoff_wp.header.stamp = now
            self.pose_pub.publish(self.takeoff_wp)
            return
        
        if self.fsm_state == FlightState.MISSION:
            self.mission_wp.header.stamp = now
            self.pose_pub.publish(self.mission_wp)
            return

        if self.fsm_state == FlightState.SEARCH:
            self.marker_wp.header.stamp = now
            self.pose_pub.publish(self.marker_wp)
            return
        
        #need an new one for the search
        if self.fsm_state == FlightState.TRACK:
            self.vel_pub.publish(self.last_correction)
            return
        #one for heardbeat???
        heartbeat = TwistStamped()
        heartbeat.header.stamp = now
        heartbeat.twist.linear.x = 0.0
        heartbeat.twist.linear.y = 0.0
        heartbeat.twist.linear.z = 0.0
        self.vel_pub.publish(heartbeat)
        return

    
    def fsm_controller(self):
        #publish
        self.state_handlers[self.fsm_state]()
        return

    '''
    establish connectiopn with FC 
    begin publish offboard signal (heartbeat)
    '''
    def run_unknown(self):
        #self.publish_velocity_setpoint()
        self.setpoint_counter += 1

        if self.setpoint_counter <= 20: return
        if not self.current_state.connected: return
        self.transition("connection_established")


    def on_preflight_start(self):
        self.setpoint_counter = 0
        self.mission_req_pending = False
        self.mission_started = False
        #set takeoff waypoint to a x meters above the curent position (home)
        self.tekeoff_wp = self.current_pos
        self.tekeoff_wp.pose.position.z += 3
        return
    '''
    ensure connection with flight controller
    set flight mode to offboard
    arm drone
    '''
    def run_preflight(self):
        #self.publish_velocity_setpoint()
        self.setpoint_counter+=1

        #wait until offboard conditions are satisfied
        if self.setpoint_counter <= 20 :return

        #wait until mission is recieved
        if not self.mission_started:
            #self.get_logger().info("waiting for mission to be recieved")
            self.mission_request(WaypointRequest.Request.MISSION_START)
            return

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
        self.pos_reached_time = None
        #probe takoff position (x meters above current pos)
        return

    '''
    takeoff verticaly
    '''
    def run_takeoff(self):

        #self.check_position_reached()
        if self.waypoint_reached(self.takeoff_wp.pose, self.current_pos.pose, self.pos_tolerance):
            self.pos_reached_time = self.get_clock().now() if self.pos_reached_time is None else self.pos_reached_time
        else:
            self.pos_reached_time = None
        
        #self.get_logger().info(f"position {abs(self.current_pos.pose.position.z)} reached for {self.takeoff_position_reached_counter} ticks")
        if (self.pos_reached_time is not None
                and (self.get_clock().now() - self.pos_reached_time) >= Duration(seconds=5)):
            self.transition("takeoff_complete")


    def on_mission_start(self):
        self.waypoint_req_pending = False
        #self.waypoint_reached = False
        self.mission_complete = False
        self.mission_wp.pose = self.takeoff_wp.pose
        self.takeoff_position_reached_counter = 0
        self.pos_reached_time = None

        return

    '''
    go to a predecided position where the marker is visible
    '''
    def run_mission(self):
        # check if mission complete
        if self.mission_complete:
            #self.get_logger().info(f"mission complete going to {self.takeoff_wp.pose}")
            self.transition("mission_complete")
            return
        
        # chech if a new wp is reached
        if self.waypoint_reached(self.mission_wp.pose,self.current_pos.pose, self.pos_tolerance):
            self.pos_reached_time = self.get_clock().now() if self.pos_reached_time is None else self.pos_reached_time
        else:
            self.pos_reached_time = None

        # make sure positin is stable
        if (self.pos_reached_time is not None
                and (self.get_clock().now() - self.pos_reached_time) >= Duration(seconds=2)):

            self.mission_request(WaypointRequest.Request.NEXT_WAYPOINT)
            #self.get_logger().info("Waypoint reached requesting next")
            return
        
        #if marker is detected transition to tracking
        stale = self.get_clock().now() - Time.from_msg(self.last_correction.header.stamp) > rclpy.duration.Duration(seconds=0.1)
        '''
        if self.detection and not stale:
            self.transition("marker_found")
        '''
        return

    def on_search_start(self):
        self.marker_wp_recieved = False
        self.marker_req_pending = False
        self.marker_wp = PoseStamped()
        self.marker_wp.pose = self.mission_wp.pose    #to prevent kick before callback returns
        marker_req = Landing.Request()
        marker_req.request = Landing.Request.SPAWN_MARKER  #needs to change
        self.get_logger().info("requesting marker coordinates")
        future = self.marker_srv.call_async(marker_req)
        future.add_done_callback(self.marker_coordinates_recieved_cb)
        return


    def marker_coordinates_recieved_cb(self,responce:Landing.Response):
        result = responce.result()
        self.marker_wp.pose.position.x = result.position.x
        self.marker_wp.pose.position.y = result.position.y
        self.marker_wp.pose.position.z = result.position.z + .5   #parametrise this later
        self.get_logger().info(f"marker coordinates recieved {result.position.x},{result.position.y},{result.position.z}")
        self.marker_wp_recieved = True
        return

    '''
    fly to the point send by the trafic controller (add some height for safety)
    if no marker is located begin a search pattern
    if no marker is detected after a certain period of time recent resent
    '''
    def run_search(self):
        if not self.marker_wp_recieved:
            return

        if not self.waypoint_reached(self.marker_wp.pose, self.current_pos.pose, self.pos_tolerance):
            #self.get_logger().info("landing waypoint reached")
            #trigger search
            return

        if self.detection:
            #self.get_logger().info("marker detected, transitioning to tracking")
            self.transition("marker_detected")
            return
        
        return

    def on_tracking_start(self):
        self.pos_reached_time = None
        self.du_stable = None
        self.landing_threshold = 0.13

        msg = SetPose.Request()
        msg.target_pose.position = self.tracking_offset.pose.position
        future = self.set_controler_target_srv.call_async(msg)
        future.add_done_callback(self.set_controller_target_cb)

        self.get_logger().info("tracking started, controller target set to 0,0,1")
        return 

    def set_controller_target_cb(self,future: SetPose.Response):
        return
    

    def run_tracking(self):
        '''
        Track the marker and align the drone with the target position.

        - If the marker is lost, transition to SEARCH via marker_lost.
        - If correction messages become stale, transition to marker_lost.
        - Check whether the drone remains within the target position tolerance.
        - If the drone stays within tolerance for approximately 2 seconds,
          transition decsent closer to the landing target slowly.
        - when the apprach is stable and the distance is safe send a disarm signal to land
        '''
        stale = self.get_clock().now() - Time.from_msg(self.last_correction.header.stamp) > rclpy.duration.Duration(seconds=0.1)
        # marker is lost, transition away from tracking
        if not self.detection:
            self.transition("marker_lost")
            return

        # look if controller msgs are stale
        if stale:
            self.transition("marker_lost") 
            return

        # check the relative movement
        if self.waypoint_reached(self.tracking_offset.pose, self.pos_error.pose, self.pos_tolerance):
        #self.check_position_reached()
            self.pos_reached_time = self.get_clock().now() if self.pos_reached_time is None else self.pos_reached_time
        else:
            self.pos_reached_time = None

        # make sure positin is stable
        relative_pos_stable = (self.pos_reached_time is not None
                and (self.get_clock().now() - self.pos_reached_time) >= Duration(seconds=2))

        relative_vel_stable = (self.du_stable is not None
                and (self.get_clock().now() - self.du_stable) >= Duration(seconds=2))

        # check stabilyty
        if relative_pos_stable and relative_vel_stable:
            
            # get closer
            self.tracking_offset.pose.position.z *= 1/2
            if self.pos_error.pose.position.z < self.landing_threshold:
                self.transition('land_ready')
            # self.transition('tracking_stable')
                        
        return

    def on_descending_start(self):
        self.descenting = False
        self.dec_mode_pending = False

        msg = SetPose.Request()
        self.tracking_offset.pose.position.z = 0.13
        msg.target_pose.position = self.tracking_offset.pose.position
        future = self.set_controler_target_srv.call_async(msg)
        future.add_done_callback(self.set_controller_target_cb)

        self.get_logger().info("tracking started, controller target set to 0,0,.13")
        return

    def run_descent(self):
        '''
        Perform the final descent toward the marker.

        - If the marker is lost, transition to marker_lost.
        - If correction messages become stale, transition to marker_lost.
        - Check whether the horizontal position error remains within the
        descent correction threshold.
        - Start descent when the drone is horizontally stable.
        - Stop descent if the drone is no longer horizontally stable.
        - When the drone is horizontally stable and sufficiently close to
        the landing pad, transition to ready_for_landing.
        '''
        # final approach to marker and transition to towtchdown
        stale = self.get_clock().now() - Time.from_msg(self.last_correction.header.stamp) > rclpy.duration.Duration(seconds=0.1)
        # marker is lost, transition away from tracking
        if not self.detection:
            self.transition("marker_lost")
            return

        # look if controller msgs are stale
        if stale:
            self.transition("marker_lost")    
            return

        # get into position


        return

    def on_touchdown_start(self):
        self.landing = False
        self.land_req_pending = False
        # send disarm request
        srv = CommandLong.Request()
        srv.command = 400
        srv.param2 = 21196.0
        future = self.command_srv.call_async(srv)
        future.add_done_callback(self.disarm_cb)
        # hope for the best
        return

    def disarm_cb(self,response):

        return
    '''
    requests landing and waits for the drone to land
    this state is entered after a go/no go decision is made and land the drone regadless of the marker detection
    it is assumed that the marker is too close for major errors to accur
    '''
    def run_touchdown(self):
        # check if relative position is good
        self.set_land()
        # request disarm and land
        # if self.current_extended_state.landed_state == 1:
            # self.transition("restart_mission")
        # wait for user input to restart the mission
        return


    '''
    ##################################################################
    /                        HELPERS                                 /
    ##################################################################
    '''
    def waypoint_reached(self, target: Pose, current:Pose, threshold: float) -> bool:
        return (
            abs(target.position.x - current.position.x) < threshold
            and abs(target.position.y - current.position.y) < threshold
            and abs(target.position.z - current.position.z) < threshold
        )
    
    

def main(args=None):
    rclpy.init(args=args)
    mission_controller = Mission_Controller()
    rclpy.spin(mission_controller)
    mission_controller.destroy_node()
    rclpy.shutdown()