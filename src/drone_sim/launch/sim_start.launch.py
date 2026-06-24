'''
This launch file starts the simulation with Gazebo, bridges, and ArUco marker detection.
It sets up the necessary bridges to connect Gazebo to ROS 2 and initializes marker generation.
'''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    drone_sim_path = get_package_share_directory('drone_sim')

    aruco_model_path = os.path.join(
        drone_sim_path,
        'models',
        'aruco_0',
        'model.sdf'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='default',
        description='Gazebo world name'
    )

    marker_x_arg = DeclareLaunchArgument(
        'marker_x',
        default_value='0.0',
        description='Marker X position'
    )

    marker_y_arg = DeclareLaunchArgument(
        'marker_y',
        default_value='0.0',
        description='Marker Y position'
    )

    marker_z_arg = DeclareLaunchArgument(
        'marker_z',
        default_value='0.02',
        description='Marker Z position'
    )
    # Check if Gazebo camera image topic exists
    gazebo_image_check = ExecuteProcess(
        cmd=['bash', '-c', 'while ! gz topic -l 2>/dev/null | grep -q "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"; do echo "Waiting for camera image topic..."; sleep 1; done'],
        name='gazebo_image_check',
        output='screen',
    )

    # Check if Gazebo camera info topic exists
    gazebo_info_check = ExecuteProcess(
        cmd=['bash', '-c', 'while ! gz topic -l 2>/dev/null | grep -q "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info"; do echo "Waiting for camera info topic..."; sleep 1; done'],
        name='gazebo_info_check',
        output='screen',
    )

    gazebo_world_check = ExecuteProcess(
        cmd=['bash', '-c', 'while ! gz topic -l 2>/dev/null | grep -q "/world/default/*"; do echo "Waiting for Gazebo world..."; sleep 1; done'],
        name='gazebo_world_check',
        output='screen',
    )

    spawn_aruco_marker = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', LaunchConfiguration('world'),
            '-name', 'aruco_marker_0',
            '-file', aruco_model_path,
            '-x', LaunchConfiguration('marker_x'),
            '-y', LaunchConfiguration('marker_y'),
            '-z', LaunchConfiguration('marker_z'),
        ],
        output='screen'
    )

    # Camera image bridge (Gazebo to ROS 2)
    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_image_bridge',
        output='screen',
        arguments=[
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        remappings=[
            (
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image',
                '/gz_camera/image_raw'
            )
        ]
    )

    # Camera info bridge (Gazebo to ROS 2)
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_info_bridge',
        output='screen',
        arguments=[
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[
            (
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info',
                '/gz_camera/camera_info'
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        marker_x_arg,
        marker_y_arg,
        marker_z_arg,
        gazebo_world_check,
        gazebo_image_check,
        gazebo_info_check,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_world_check,
                on_exit=[spawn_aruco_marker]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_image_check,
                on_exit=[image_bridge]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_info_check,
                on_exit=[camera_info_bridge]
            )
        ),
        
    ])