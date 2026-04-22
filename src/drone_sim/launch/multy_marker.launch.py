from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('drone_sim')

    world = os.path.join(pkg_path, 'worlds', 'multy_aruco_test.sdf')

    gz = ExecuteProcess(
        cmd=['gz', 'sim', world],
        output='screen'
    )

    return LaunchDescription([
        gz
    ])
