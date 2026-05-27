from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    drone_sim_path = get_package_share_directory('drone_sim')
    drone_vision_path = get_package_share_directory('drone_vision')
    drone_control_path = get_package_share_directory('drone_control')

    # -------------------------
    # Simulation + bridges + marker
    # -------------------------
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                drone_sim_path,
                'launch',
                'sim_start.launch.py'
            )
        )
    )

    # -------------------------
    # Vision (ArUco detector)
    # -------------------------
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                drone_vision_path,
                'launch',
                'camera_packet.launch.py'
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
        sim_launch,
        vision_launch,
        control_launch,
    ])
