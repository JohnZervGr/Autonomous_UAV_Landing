#!/usr/bin/env bash

set -e

ROS_WS=~/Documents/Drone_Landing/workspace
PX4_DIR=~/Documents/Drone_Landing/PX4-Autopilot
WORLD_NAME=aruco_test_world

echo "Starting PX4 + Gazebo + ArUco world..."

# ---- Terminal 1: Gazebo ----
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ${ROS_WS}/install/setup.bash
source ${PX4_DIR}/build/px4_sitl_default/rootfs/gz_env.sh

gz sim \$(ros2 pkg prefix drone_sim)/share/drone_sim/worlds/${WORLD_NAME}.sdf

exec bash
"

# Give Gazebo time to boot
sleep 5

# ---- Terminal 2: PX4 ----
gnome-terminal -- bash -c "
cd ${PX4_DIR}

source /opt/ros/jazzy/setup.bash
source ${ROS_WS}/install/setup.bash
source ${PX4_DIR}/build/px4_sitl_default/rootfs/gz_env.sh

PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=${WORLD_NAME} PX4_GZ_MODEL=x500_mono_cam make px4_sitl gz_x500_mono_cam

exec bash
"

echo "Simulation started."