from landing_interfaces.action import Waypoint
import rclpy
from rclpy.node import Node
import yaml
from std_srvs.srv import SetBool 

from landing_interfaces.srv import WaypointRequest



class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.get_logger().info("initialising mission node")
        self.mission_ready = False

        #find mission file
        self.declare_parameter("mission_file", "src/drone_control/config/mission_test.yaml")
        mission_file = self.get_parameter("mission_file").value

        #read mission file
        self.get_logger().info(f"Using mission file: {mission_file}")
        with open(mission_file, 'r') as f:
            self.mission_data = yaml.safe_load(f)

        #load mission parameters
        self.mission_name = self.mission_data['mission']['name']
        self.min_altitude = self.mission_data['mission']['min_altitude']
        self.max_altitude = self.mission_data['mission']['max_altitude']

        #load mission waypoints
        self.waypoints = self.mission_data['waypoints']
        self.waypoints_no = len(self.waypoints) if self.waypoints is not None else 0
        self.current_waypoint = 0

        self.get_logger().info(f"Mission name: {self.mission_name}")
        self.get_logger().info(f"Mission min altitude: {self.min_altitude}")
        self.get_logger().info(f"Mission max altitude: {self.max_altitude}")
        self.get_logger().info(f"Number of waypoints: {self.waypoints_no}")
        self.get_logger().info("mission file loaded successfully")

        #set up service
        self.mission_service = self.create_service(WaypointRequest,
                                                   "/mission/control",
                                                    self.mission_handler_cb)


        self.get_logger().info("mission node initialised")
        self.mission_ready = True


    def mission_handler_cb(self,req:WaypointRequest.Request,res:WaypointRequest.Response):

        if not self.mission_ready:
            res.status = WaypointRequest.Response.ERROR
            return res
        
        #start or reset the mission 
        if req.mission_phase == WaypointRequest.Request.MISSION_START:

            #set mission to the first waypoint
            self.current_waypoint = 0
            self.get_logger().info(f"Start srv recieved setting waypoint {self.current_waypoint}")

            #respond suscefully
            res.status = WaypointRequest.Response.MISSION_STARTED
            return res
        
        #send the next waypoint in the list
        if req.mission_phase == WaypointRequest.Request.NEXT_WAYPOINT:

            #check if the mission is over
            if self.current_waypoint >= self.waypoints_no:
                self.get_logger().info("mission complete")
                res.status = WaypointRequest.Response.MISSION_COMPLETE
                return res
            
            #load the next wp
            current_wp = self.waypoints[self.current_waypoint]
            res.status = WaypointRequest.Response.WAYPOINT
            res.waypoint.position.x = current_wp['position']['x']
            res.waypoint.position.y = current_wp['position']['y']
            res.waypoint.position.z = current_wp['position']['z']

            #update
            self.get_logger().info(f"Next waypoint request resived sending waypoint: {self.current_waypoint}")
            self.current_waypoint+=1
            return res

        res.status = WaypointRequest.Response.ERROR
        return res



def main(args=None):
    rclpy.init(args=args)
    mission_node = MissionNode()
    rclpy.spin(mission_node)
    mission_node.destroy_node()
    rclpy.shutdown()