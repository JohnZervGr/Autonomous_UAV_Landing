from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():

    pkg_name = "drone_control"

    pid_param_file = os.path.join(
        get_package_share_directory(pkg_name),
        "config",
        "pid_params.yaml"
    )

    controller_node = Node(
        package=pkg_name,
        executable="controller_node",
        name="pid_controller",
        output="screen",
        parameters=[pid_param_file,
                    {"DEBUG": False},
                    ],
    )

    return LaunchDescription([
        controller_node
    ])