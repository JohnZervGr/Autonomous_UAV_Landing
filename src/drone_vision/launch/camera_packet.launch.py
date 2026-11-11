'''
Launch file for the camera packet node, which detects ArUco markers in the camera feed and publishes their poses.
'''
from launch import LaunchDescription
from launch_ros.actions import DeclareLaunchArgument, Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='0'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='1.0'
    )

    dictionary_arg = DeclareLaunchArgument(
        'dictionary',
        default_value='DICT_4X4_50'
    )

    aruco_detector_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'marker_id': 0,
            'marker_size': 1.20,
            'reference_frame': 'camera_link',
            'dictionary': 'DICT_4X4_50',
            'camera_frame': 'camera_link',
            'marker_frame': 'aruco_marker',
        }],
        remappings=[
            ('/image', '/gz_camera/image_raw'),
            ('/camera_info', '/gz_camera/image_info'),
        ]
    )

    return LaunchDescription([
        marker_id_arg,
        marker_size_arg,
        dictionary_arg,
        aruco_detector_node
    ])