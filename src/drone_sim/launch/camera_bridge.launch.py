from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_image_bridge',
        output='screen',
        arguments=[
            '/world/aruco_test_world/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        remappings=[
            (
                '/world/aruco_test_world/model/x500_mono_cam_0/link/camera_link/sensor/camera/image',
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
            '/world/aruco_test_world/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[
            (
                '/world/aruco_test_world/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info',
                '/gz_camera/camera_info'
            )
        ]
    )

    return LaunchDescription([
        image_bridge,
        camera_info_bridge,
    ])