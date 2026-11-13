import re
import subprocess
import os

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.logging import get_logger

from ament_index_python.packages import get_package_share_directory


logger = get_logger('aruco_marker_spawner')


# ==========================================
# 1. DISCOVER ACTIVE GAZEBO WORLDS
# ==========================================

def discover_active_worlds():

    try:
        result = subprocess.run(
            ['gz', 'service', '-l'],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode != 0:
            logger.error('Failed to discover Gazebo worlds.')
            return []

        worlds = set()

        for service in result.stdout.splitlines():

            match = re.match(
                r'/world/([^/]+)/',
                service.strip()
            )

            if match:
                worlds.add(match.group(1))

        active_worlds = sorted(worlds)

        logger.info(f'Active worlds: {active_worlds}')

        return active_worlds

    except Exception as e:

        logger.error(f'World discovery failed: {e}')

        return []


# ==========================================
# 2. SPAWN MARKERS
# ==========================================

def spawn_markers(context):

    drone_sim_path = get_package_share_directory('drone_sim')

    aruco_model_path = os.path.join(
        drone_sim_path,
        'models',
        'aruco_0_small',
        'model.sdf'
    )

    # Launch parameters
    marker_id = LaunchConfiguration('marker_id').perform(context)
    #marker_size = LaunchConfiguration('marker_size').perform(context)
    marker_x = LaunchConfiguration('marker_x').perform(context)
    marker_y = LaunchConfiguration('marker_y').perform(context)
    marker_z = LaunchConfiguration('marker_z').perform(context)

    # Discover worlds
    worlds = discover_active_worlds()

    if not worlds:
        logger.warning('No active Gazebo worlds found.')
        return []

    spawn_actions = []

    for world in worlds:

        marker_name = f'aruco_marker_{marker_id}_{world}'

        logger.info(
            f'Spawning marker ID {marker_id} '
            f'in world {world} '
            f'at ({marker_x}, {marker_y}, {marker_z})'
        )

        spawn_actions.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', world,
                    '-name', marker_name,
                    '-file', aruco_model_path,
                    '-x', marker_x,
                    '-y', marker_y,
                    '-z', marker_z,
                ],
                output='screen'
            )
        )

    return spawn_actions


# ==========================================
# 3. LAUNCH DESCRIPTION
# ==========================================

def generate_launch_description():


    marker_node = Node(
        package='drone_sim',
        executable='marker_spawner',
        name='marker_node',
        parameters=[
            {'x':LaunchConfiguration('marker_x')},
            {'y':LaunchConfiguration('marker_y')},
            {'z':LaunchConfiguration('marker_z')},
        ],
        output='screen'
    )

    marker_control_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='marker_control_bridge',
        arguments=[
            '/model/aruco_marker_0_default/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'marker_id',
            default_value='0',
            description='ArUco marker ID'
        ),
        DeclareLaunchArgument(
            'marker_x',
            default_value='0.0',
            description='Marker X position'
        ),
        DeclareLaunchArgument(
            'marker_y',
            default_value='0.0',
            description='Marker Y position'
        ),
        DeclareLaunchArgument(
            'marker_z',
            default_value='0.001',
            description='Marker Z position'
        ),
        marker_node,
        marker_control_bridge,
        OpaqueFunction(
            function=spawn_markers
        )
        

    ])