from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    drone_sim_path = get_package_share_directory('drone_sim')

    aruco_sdf = os.path.join(
        drone_sim_path,
        'models',
        'aruco_0',
        'model.sdf'
    )

    spawn_aruco = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'default',
            '-name', 'aruco_marker_0',
            '-file', aruco_sdf,
            '-x', '0',
            '-y', '0',
            '-z', '0.02'
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_aruco
    ])