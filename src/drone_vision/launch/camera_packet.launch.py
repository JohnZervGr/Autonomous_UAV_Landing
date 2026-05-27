'''
Launch file for the camera packet node, which detects ArUco markers in the camera feed and publishes their poses.
'''

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    aruco_detector_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'marker_id': 0,
            'marker_size': 1.20,
            'reference_frame': 'camera_link',
            'dictionary': 'DICT_4X4_50',
            'camera_frame': 'camera_link',
            'marker_frame': 'aruco_marker',
        }],
        remappings=[
            ('/image', '/gz_camera/image_raw'),
            ('/camera_info', '/gz_camera/camera_info'),
        ]
    )

    return LaunchDescription([
        aruco_detector_node
    ])