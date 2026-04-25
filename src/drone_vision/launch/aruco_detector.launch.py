from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/gz_camera/image_raw',
        description='ROS image topic to subscribe to'
    )

    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_image_bridge',
        output='screen',
        arguments=[
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        remappings=[
            (
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image',
                '/gz_camera/image_raw'
            )
        ]
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_info_bridge',
        output='screen',
        arguments=[
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[
            (
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info',
                '/gz_camera/camera_info'
            )
        ]
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
            ('/camera_info', '/gz_camera/camera_info'),
        ]
    )

    return LaunchDescription([
        image_topic_arg,
        image_bridge,
        camera_info_bridge,
        aruco_detector_node
    ])