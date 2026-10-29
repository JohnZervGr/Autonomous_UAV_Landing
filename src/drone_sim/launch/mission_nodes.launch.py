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

    aruco_config_path = os.path.join(
        drone_sim_path,
        'config',
        'marker.yaml'
    )

    with open(aruco_config_path, 'r') as file:
            aruco_config = yaml.safe_load(file)['marker']

    aruco_config_path = os.path.join(
            drone_sim_path,
            'config',
            'marker.yaml'
        )

    aruco_model_path = os.path.join(
        drone_sim_path,
        'models',
        aruco_config['model'],
        'model.sdf'
    )
    
    spawn_aruco_marker = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_sim',
            'create',
            '-world',
            LaunchConfiguration('world'),
            '-name',
            'aruco_marker_0',
            '-file',
            aruco_model_path,
            '-x',
            str(aruco_config['position']['x']),
            '-y',
            str(aruco_config['position']['y']),
            '-z',
            str(aruco_config['position']['z'])
        ],
        output='screen'
    )


    mission_node = Node(
        package='drone_control',
        executable='mission_node',
        name='mission_node',
        output='screen'
    )

    marker_node = Node(
        package='drone_sim',
        executable='marker_spawner',
        name='marker_node',
        parameters=[
            {'x':aruco_config['position']['x']},
            {'y':aruco_config['position']['y']},
            {'z':aruco_config['position']['z']},
        ],
        output='screen'
    )


    return LaunchDescription([
        world_arg,
        spawn_aruco_marker,
        marker_node,
        mission_node,
          

    ])