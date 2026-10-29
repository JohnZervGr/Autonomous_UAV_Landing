from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def generate_launch_description():


    gazebo_world_check = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'while ! gz topic -l | grep -q "/world/"; do '
            'echo "Waiting for Gazebo world..."; sleep 1; done'
        ],
        output='screen'
    )


    gazebo_image_check = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'while ! gz topic -l | grep -q "camera/image"; do '
            'echo "Waiting for camera image..."; sleep 1; done'
        ],
        output='screen'
    )

    mission_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drone_sim'),
                'launch',
                'mission_nodes.launch.py'
            )
        )
    )

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drone_sim'),
                'launch',
                'mavros_setup.launch.py'
            )
        )
    )


    return LaunchDescription([

        # arguments
        #world_arg,

        # start sequence
        gazebo_world_check,
        gazebo_image_check,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_world_check,
                on_exit=[
                    mission_launch,
                    mavros_launch
                ]
            )
        ),


    ])