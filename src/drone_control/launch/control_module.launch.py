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
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',
        description='MAVROS FCU URL for PX4 SITL'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='Optional GCS proxy URL'    
    )

    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='Target system id'
    )

    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='Target component id'
    )

    # MAVROS launch
    mavros_launch = os.path.join(
        get_package_share_directory('mavros'),
        'launch',
        'px4.launch'
    ) 

    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
        }.items()
    )

    # mission controller node
    mission_controller_node = Node(
        package='drone_control',
        executable='mission_controller_node',
        name='mission_controller',
        output='screen',
    )

    tracker_node = Node(
        package='drone_control',
        executable='camera_tracker_node',
        name='camera_tracker',
        output='screen', 
    )
    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        mavros_node,
        mission_controller_node,
        tracker_node,
    ])