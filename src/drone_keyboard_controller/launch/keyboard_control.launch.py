from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',
        description='Connection URL for the flight controller'
    )

    teleop_speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.5',
        description='Initial linear speed for teleop_twist_keyboard'
    )

    teleop_turn_arg = DeclareLaunchArgument(
        'turn',
        default_value='1.0',
        description='Initial angular speed for teleop_twist_keyboard'
    )

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavros'),
                'launch',
                'px4.launch'
            )
        ),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url')
        }.items()
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'speed': LaunchConfiguration('speed'),
            'turn': LaunchConfiguration('turn'),
        }]
    )

    controller_node = Node(
        package='drone_keyboard_controller',
        executable='twist_controller',
        name='twist_controller',
        output='screen'
    )

    return LaunchDescription([
        fcu_url_arg,
        teleop_speed_arg,
        teleop_turn_arg,
        mavros_launch,
        teleop_node,
        controller_node,
    ])