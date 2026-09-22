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

    # -------------------------
    # Arguments
    # -------------------------

    

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='default',
        description='Gazebo world name'
    )

    # -------------------------
    # Gazebo readiness checks
    # -------------------------

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


    gazebo_info_check = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'while ! gz topic -l | grep -q "camera_info"; do '
            'echo "Waiting for camera info..."; sleep 1; done'
        ],
        output='screen'
    )


    # -------------------------
    # Spawn marker
    # -------------------------

    aruco_config_path = os.path.join(
        drone_sim_path,
        'config',
        'marker.yaml'
    )

    with open(aruco_config_path, 'r') as file:
        aruco_config = yaml.safe_load(file)['marker']

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


    # -------------------------
    # Gazebo bridges
    # -------------------------

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


    # -------------------------
    # Launch sequence
    # -------------------------

    return LaunchDescription([

        # arguments
        world_arg,

        # start sequence
        gazebo_world_check,
        gazebo_image_check,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_world_check,
                on_exit=[
                    spawn_aruco_marker,
                    mavros_node
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_image_check,
                on_exit=[
                    bridge
                ]
            )
        ),
    ])