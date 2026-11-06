import re
import time
import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.logging import get_logger

from launch_ros.actions import Node

logger = get_logger('gazebo_bridge_discovery')


# ==================================================
# 1. GAZEBO TOPIC DISCOVERY
# ==================================================

def get_gazebo_topics():
    try:
        result = subprocess.run(
            ['gz', 'topic', '-l'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode != 0:
            return []
        return [
            topic.strip()
            for topic in result.stdout.splitlines()
            if topic.strip()
        ]
    except Exception as e:
        logger.warning(
            f'Gazebo topic discovery failed: {e}'
        )
        return []


# ==================================================
# 2. GAZEBO MESSAGE TYPE DISCOVERY
# ==================================================

def get_gazebo_topic_type(topic):
    try:
        result = subprocess.run(
            ['gz', 'topic', '-i', '-t', topic],
            capture_output=True,
            text=True,
            timeout=5
        )
        output = result.stdout + result.stderr
        match = re.search(
            r'gz\.msgs\.[A-Za-z0-9_]+',
            output
        )
        if match:
            return match.group(0)
    except Exception as e:
        logger.warning(
            f'Could not inspect {topic}: {e}'
        )
    return None

# ==================================================
# 3. WAIT FOR GAZEBO
# ==================================================
def wait_for_gazebo():
    logger.info('Waiting for Gazebo camera topics...')
    timeout = 60
    start_time = time.time()
    while time.time() - start_time < timeout:
        topics = get_gazebo_topics()
        camera_topics = [
            topic for topic in topics
            if topic.startswith('/world/')
            and (
                topic.endswith('/image')
                or topic.endswith('/camera_info')
            )
        ]
        if camera_topics:
            logger.info(
                f'Found {len(camera_topics)} camera topics.'
            )
            # Allow additional cameras/worlds to initialize.
            time.sleep(3)
            return get_gazebo_topics()
        time.sleep(1)
    logger.warning(
        'Timed out waiting for Gazebo camera topics.'
    )
    return []


# ==================================================
# 4. DISCOVER BRIDGE CONFIGURATIONS
# ==================================================
def discover_bridges():
    topics = wait_for_gazebo()
    bridges = []
    discovered_worlds = set()

    for topic in topics:
        if not topic.startswith('/world/'):
            continue

        # Determine Gazebo and ROS message types.
        if topic.endswith('/image'):
            ros_type = 'sensor_msgs/msg/Image'
            gz_type = 'gz.msgs.Image'

        elif topic.endswith('/camera_info'):
            ros_type = 'sensor_msgs/msg/CameraInfo'
            gz_type = 'gz.msgs.CameraInfo'

        else:
            continue

        # Verify the actual Gazebo message type.
        actual_type = get_gazebo_topic_type(topic)
        if actual_type != gz_type:
            logger.warning(
                f'Skipping {topic}: '
                f'expected {gz_type}, got {actual_type}'
            )
            continue

        # Extract world name.
        parts = topic.split('/')
        if len(parts) < 4:
            continue
        world_name = parts[2]
        discovered_worlds.add(world_name)

        # Generate ROS topic name.
        if topic.endswith('/image'):
            ros_topic = '/gz_camera/image_raw'
        elif topic.endswith('/camera_info'):
            ros_topic = '/gz_camera/image_info'

        # Generate unique bridge node name.
        bridge_name = re.sub(
            r'[^a-zA-Z0-9_]',
            '_',
            'bridge_' + topic.strip('/')
        )

        # Store bridge configuration.
        bridges.append({
            'name': bridge_name,
            'world': world_name,
            'gz_topic': topic,
            'ros_topic': ros_topic,
            'ros_type': ros_type,
            'gz_type': gz_type,
        })

    # Discovery summary.
    logger.info('===================================')
    logger.info(
        f'Discovered worlds: {sorted(discovered_worlds)}'
    )
    logger.info(
        f'Discovered bridges: {len(bridges)}'
    )
    logger.info('===================================')
    return bridges


# ==================================================
# 5. CREATE NORMAL ROS 2 BRIDGE NODES
# ==================================================

def create_bridge_nodes(bridges):

    nodes = []
    for bridge in bridges:

        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=bridge['name'],
            output='screen',
            arguments=[
                f"{bridge['gz_topic']}"
                f"@{bridge['ros_type']}"
                f"@{bridge['gz_type']}"
            ],
            remappings=[
                (
                    bridge['gz_topic'],
                    bridge['ros_topic']
                )
            ])
        nodes.append(bridge_node)
        logger.info(
            f"Creating bridge: "
            f"{bridge['gz_topic']} -> {bridge['ros_topic']}"
        )
    return nodes

# ==================================================
# 6. LAUNCH SETUP
# ==================================================

def setup_bridges(context):
    bridges = discover_bridges()
    bridge_nodes = create_bridge_nodes(bridges)
    return bridge_nodes

# ==================================================
# 7. LAUNCH DESCRIPTION
# ==================================================

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(
            function=setup_bridges
        )
    ])