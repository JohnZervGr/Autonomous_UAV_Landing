from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch arguments -------------------------------------------------
    args = [
        DeclareLaunchArgument('model_file', default_value='/models/aruco_0_small/model.sdf',
                              description='Absolute path to the marker SDF file'),
        DeclareLaunchArgument('model_name', default_value='moving_aruco_marker',
                              description='Entity name in Gazebo (must match the cmd_vel topic)'),
        DeclareLaunchArgument('world', default_value='windy',
                              description='Name of the running Gazebo world'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.01'),
        DeclareLaunchArgument('move', default_value='false',
                              description='Publish a constant Twist to the marker from here'),
        DeclareLaunchArgument('vx', default_value='0.5', description='Constant x velocity (m/s)'),
    ]

    model_name = LaunchConfiguration('model_name')
    cmd_vel_topic = ['/model/', model_name, '/cmd_vel']

    # --- Spawn the marker into the running world --------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', LaunchConfiguration('world'),
            '-file', LaunchConfiguration('model_file'),
            '-name', model_name,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen',
    )

    # --- Bridge cmd_vel (ROS -> Gazebo) -----------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/', model_name,
             '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen',
    )

    # --- Optional: publish a constant velocity ----------------------------
    mover = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('move')),
        cmd=[
            'ros2', 'topic', 'pub', '-r', '20',
            cmd_vel_topic, 'geometry_msgs/msg/Twist',
            ['{linear: {x: ', LaunchConfiguration('vx'), ', y: 0.0, z: 0.0}, '
             'angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        ],
        output='screen',
    )

    return LaunchDescription(args + [spawn, bridge, mover])