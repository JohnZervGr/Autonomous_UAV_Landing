from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------------------------
    # Launch arguments
    # -------------------------
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/gz_camera/image_raw',
        description='Camera image topic'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/gz_camera/camera_info',
        description='Camera info topic'
    )

    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='0',
        description='Target ArUco marker ID'
    )

    dictionary_arg = DeclareLaunchArgument(
        'dictionary',
        default_value='DICT_4X4_50',
        description='ArUco dictionary'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.1',
        description='ArUco marker size in meters'
    )

    # -------------------------
    # ArUco detector node
    # -------------------------
    aruco_detector_node = Node(
        package='drone_vision',
        executable='detector',
        name='detector',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'marker_id': LaunchConfiguration('marker_id'),
            'dictionary': LaunchConfiguration('dictionary'),
            'marker_size': LaunchConfiguration('marker_size'),
        }]
    )

    # -------------------------
    # Guidance node
    # -------------------------
    guidance_node = Node(
        package='drone_vision',
        executable='guidance',
        name='guidance_node',
        output='screen',
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        marker_id_arg,
        dictionary_arg,
        marker_size_arg,
        aruco_detector_node,
        guidance_node
    ])