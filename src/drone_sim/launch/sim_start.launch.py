from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    drone_sim_path = get_package_share_directory('drone_sim')

    aruco_model_path = os.path.join(
        drone_sim_path,
        'models',
        'aruco_0',
        'model.sdf'
    )

    # -------------------------
    # Arguments
    # -------------------------

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='default',
        description='Gazebo world name'
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

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',
        description='PX4 SITL FCU URL'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value=''
    )

    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1'
    )

    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1'
    )


    # -------------------------
    # Gazebo readiness checks
    # -------------------------

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


    gazebo_info_check = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'while ! gz topic -l | grep -q "camera_info"; do '
            'echo "Waiting for camera info..."; sleep 1; done'
        ],
        output='screen'
    )


    # -------------------------
    # Spawn marker
    # -------------------------

    spawn_aruco_marker = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_sim',
            'create',
            '-world',
            LaunchConfiguration('world'),
            '-name',
            'aruco_marker_0',
            '-file',
            aruco_model_path,
            '-x',
            LaunchConfiguration('marker_x'),
            '-y',
            LaunchConfiguration('marker_y'),
            '-z',
            LaunchConfiguration('marker_z')
        ],
        output='screen'
    )


    # -------------------------
    # Gazebo bridges
    # -------------------------

    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_image_bridge',
        arguments=[
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        remappings=[
            (
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image',
                '/gz_camera/image_raw'
            )
        ],
    )


    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_info_bridge',
        arguments=[
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[
            (
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info',
                '/gz_camera/camera_info'
            )
        ],
    )


    # -------------------------
    # MAVROS
    # -------------------------

    mavros_launch = os.path.join(
        get_package_share_directory('mavros'),
        'launch',
        'px4.launch'
    )


    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
        }.items()
    )


    # -------------------------
    # Launch sequence
    # -------------------------

    return LaunchDescription([

        # arguments
        world_arg,
        marker_x_arg,
        marker_y_arg,
        marker_z_arg,

        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,


        # start sequence
        gazebo_world_check,
        gazebo_image_check,
        gazebo_info_check,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_world_check,
                on_exit=[
                    spawn_aruco_marker,
                    mavros_node
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_image_check,
                on_exit=[
                    image_bridge
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_info_check,
                on_exit=[
                    camera_info_bridge
                ]
            )
        ),
    ])