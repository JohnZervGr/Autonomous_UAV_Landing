#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from landing_interfaces.srv import Landing
from ros_gz_interfaces.srv import SpawnEntity

'''
the primary function of this node is to recieve a landing request (service)
and spawn a landing pad while responding with the location
the location on the responce is expected to have some error to simlulate gps error
'''
class Marker_Spawner(Node):
    def __init__(self):
        super().__init__('landing_pad_spawn_node')

        self.world = "default/world"

        self.path_to_sdf = "/home/zerv/Documents/Drone_Landing/workspace/src/drone_sim/models/aruco_0/model.sdf"
        self.get_logger().info(f"SDF path = {self.path_to_sdf}")

        self.marker_position = Point()
        self.marker_position.x = self.declare_parameter('x', 0.0).value
        self.marker_position.y = self.declare_parameter('y', 0.0).value
        self.marker_position.z = self.declare_parameter('z', 0.05).value  # a bit above sureface so it is visible

        self.marker_id = self.declare_parameter('marker_id', 1).value

        #create landing service server
        self.create_service(Landing,
                            'landing/request',
                            self.landing_cb)

        self.spawn_marker = self.create_client(SpawnEntity,
                                               '/world/default/create')

        self.get_logger().info("node initialiasation complete")
        self.get_logger().info(f"marker position = {self.marker_position.x},{self.marker_position.y},{self.marker_position.z}")


    def landing_cb(self,req:Landing.Request,responce:Landing.Response):
        #find suitable locations for marker
        responce.position = self.marker_position  #add randomization to the responce
        responce.marker_id = self.marker_id       #for future if spawning muliple markers
        self.get_logger().info(f"request recieved:{req.request}")
        '''if req.request == Landing.Request.SPAWN_MARKER:
            self.get_logger().info("spawning marker...")
            #spawn marker 
            spawn_Req = SpawnEntity.Request()
            spawn_Req.entity_factory.name = "landing_marker"
            spawn_Req.entity_factory.pose.position = self.marker_position
            spawn_Req.entity_factory.sdf_filename = self.path_to_sdf
            spawn_Req.entity_factory.allow_renaming = True
            #self.get_logger().info(f"marker spawn request sent: {spawn_Req}")
            future = self.spawn_marker.call_async(spawn_Req)'''
        #respond with marker location
        return responce

def main(args=None):
    rclpy.init(args=args)
    marker_spawner = Marker_Spawner()
    rclpy.spin(marker_spawner)
    marker_spawner.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()