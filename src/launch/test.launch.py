from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Mavros
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url' : 'udp://:14540@localhost:14557',
                # 'tgt_system': LaunchConfiguration('tgt_system'),
                # You can add other MAVROS parameters here if needed
                # 'gcs_url': '',
                # 'tgt_component': 1,
            }]
        )
    ])