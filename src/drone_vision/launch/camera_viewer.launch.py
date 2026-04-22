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

    camera_viewer_node = Node(
        package='drone_vision',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic')
        }]
    )

    return LaunchDescription([
        image_topic_arg,
        camera_viewer_node
    ])