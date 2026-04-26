from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # -------------------------
    # Package paths
    # -------------------------
    drone_sim_path = get_package_share_directory('drone_sim')

    aruco_model_path = os.path.join(
        drone_sim_path,
        'models',
        'aruco_0',
        'model.sdf'
    )

    # -------------------------
    # Launch arguments
    # -------------------------
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

    # -------------------------
    # Gazebo camera image bridge
    # -------------------------
    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_image_bridge',
        output='screen',
        arguments=[
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image'
            '@sensor_msgs/msg/Image'
            '@gz.msgs.Image'
        ],
        remappings=[
            (
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image',
                '/gz_camera/image_raw'
            )
        ]
    )

    # -------------------------
    # Gazebo camera info bridge
    # -------------------------
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_info_bridge',
        output='screen',
        arguments=[
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '@gz.msgs.CameraInfo'
        ],
        remappings=[
            (
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info',
                '/gz_camera/camera_info'
            )
        ]
    )

    # -------------------------
    # Spawn ArUco marker
    # -------------------------
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

    return LaunchDescription([
        world_arg,
        marker_x_arg,
        marker_y_arg,
        marker_z_arg,
        image_bridge,
        camera_info_bridge,
        spawn_aruco_marker,
    ])