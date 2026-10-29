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

    drone_sim_path = get_package_share_directory('drone_sim')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='default',
        description='Gazebo world name'
    )

    # -------------------------
    # MAVROS
    # -------------------------

    mavros_launch = os.path.join(
        get_package_share_directory('mavros'),
        'launch',
        'px4.launch'
    )

    mavros_config_path = os.path.join(
        drone_sim_path,
        'config',
        'mavros.yaml'
    )

    with open(mavros_config_path, 'r') as file:
        mavros_config = yaml.safe_load(file)['mavros']


    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch),
        launch_arguments={
            'fcu_url': mavros_config['fcu_url'],
            'gcs_url': mavros_config['gcs_url'],
            'tgt_system': str(mavros_config['target']['system']),
            'tgt_component': str(mavros_config['target']['component']),
        }.items()
    )


    world = LaunchConfiguration('world')

    camera_config_path = PathJoinSubstitution([
        drone_sim_path,
        'config',
        ['bridges_', world, '.yaml']
    ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_image_bridge',
        parameters=[{'config_file':camera_config_path}],
        output='screen'
    )


    return LaunchDescription([
        world_arg,
        mavros_node,
        bridge
    ])


