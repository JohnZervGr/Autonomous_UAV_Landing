from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    mission_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='src/drone_sim/config/mission_empty.yaml',
        description='Path to the mission YAML file'
    )

    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='0'
    )

    marker_x_arg = DeclareLaunchArgument(
        'marker_x',
        default_value='0.0'
    )

    marker_y_arg = DeclareLaunchArgument(
        'marker_y',
        default_value='0.0'
    )

    marker_z_arg = DeclareLaunchArgument(
        'marker_z',
        default_value='0.02'
    )

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

    mission_launch = Node(
        package='drone_sim',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[
            {'mission_file': LaunchConfiguration('mission_file')}
        ]
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

    bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drone_sim'),
                'launch',
                'bridge_setup.launch.py'
            )
        )
    )

    marker_spawn_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drone_sim'),
                'launch',
                'spawn_marker.launch.py'
            )
        ),
        launch_arguments={
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_x': LaunchConfiguration('marker_x'),
            'marker_y': LaunchConfiguration('marker_y'),
            'marker_z': LaunchConfiguration('marker_z'),
        }.items()
    )

    return LaunchDescription([

        # arguments
        marker_id_arg,
        marker_x_arg,
        marker_y_arg,
        marker_z_arg,
        mission_arg,

        # start sequence
        gazebo_world_check,
        gazebo_image_check,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_world_check,
                on_exit=[
                    mission_launch,
                    mavros_launch,
                    bridge_launch,
                    marker_spawn_launch
                ]
            )
        ),
    ])