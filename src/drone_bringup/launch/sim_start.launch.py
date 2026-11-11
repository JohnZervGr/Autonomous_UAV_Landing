from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    drone_sim_path = get_package_share_directory('drone_sim')
    drone_vision_path = get_package_share_directory('drone_vision')
    drone_control_path = get_package_share_directory('drone_control')

    defaults = {
        'marker_id': '0',
        'marker_size': '0.1',
        'dictionary': 'DICT_4X4_50',
        'marker_x': '1.0',
        'marker_y': '2.0',
        'marker_z': '0.01',
        'mission_file': 'src/drone_sim/config/mission_empty.yaml',
    }

    default_launch_arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]

    # -------------------------
    # Launch arguments  
    # -------------------------
    sim_args = {
        'marker_id': LaunchConfiguration('marker_id'),
        'marker_size': LaunchConfiguration('marker_size'),
        'dictionary': LaunchConfiguration('dictionary'),
        'marker_x' : LaunchConfiguration('marker_x'),
        'marker_y' : LaunchConfiguration('marker_y'),
        'marker_z' : LaunchConfiguration('marker_z'),
        'mission_file': LaunchConfiguration('mission_file')
    }

    vis_args = {
        'image_topic': LaunchConfiguration('image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'marker_id': LaunchConfiguration('marker_id'),
        'dictionary': LaunchConfiguration('dictionary'),
        'marker_size': LaunchConfiguration('marker_size'),
    }

    control_args = {
        'image_topic': LaunchConfiguration('image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'marker_id': LaunchConfiguration('marker_id'),
        'dictionary': LaunchConfiguration('dictionary'),
        'marker_size': LaunchConfiguration('marker_size'),
    }


    # -------------------------
    # Simulation + bridges + marker
    # -------------------------
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                drone_sim_path,
                'launch',
                'sim.launch.py'
            )
        ),
        launch_arguments=sim_args.items()
    )

    # -------------------------
    # Vision (ArUco detector)
    # -------------------------
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                drone_vision_path,
                'launch',
                'vision_packet.launch.py'
            )
        )
    )

    # -------------------------
    # Control (MAVROS + FSM + controller)
    # -------------------------
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                drone_control_path,
                'launch',
                'control_module.launch.py'
            )
        )
    )

    return LaunchDescription([
        *default_launch_arguments,
        #sim_args,
        #vis_args,
        #control_args,

        sim_launch,
        vision_launch,
        control_launch
                
    ])
