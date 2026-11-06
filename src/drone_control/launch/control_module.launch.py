'''
This launch file starts the MAVROS node for PX4 SITL and the mission controller node. 
It also allows to specify the MAVROS connection parameters via launch arguments.
Example usage:
$ ros2 launch drone_control control.launch.py fcu_url:=udp://:14540@
'''


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch args
    pkg_name = "drone_control"

    pid_param_file = os.path.join(
        get_package_share_directory(pkg_name),
        "config",
        "pid_params.yaml"
    )

    # MAVROS launch

    controller_node = Node(
        package=pkg_name,
        executable="controller_node",
        name="pid_controller",
        output="screen",
        parameters=[pid_param_file,
                    {"DEBUG": False},
                    ],
    )

    # mission controller node
    mission_controller_node = Node(
        package='drone_control',
        executable='mission_controller_node',
        name='mission_controller',
        output='screen',
    )

    return LaunchDescription([
        controller_node,
        mission_controller_node,
    ])