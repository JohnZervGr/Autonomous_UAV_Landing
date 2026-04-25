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

    namespace_arg = DeclareLaunchArgument(
        'mavros_namespace',
        default_value='mavros',
        description='Namespace for MAVROS'
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
            'namespace': LaunchConfiguration('mavros_namespace'),
        }.items()
    )

    # Your drone control node
    control_node = Node(
        package='drone_control',
        executable='drone_control_node',   # change to your real executable name
        name='drone_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        namespace_arg,
        mavros_node,
        control_node,
    ])