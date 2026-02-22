#!/bin/bash
# Start SLAM and ROS1 ZMQ bridge on the Jetson
# Usage: ./start_slam.sh
#
# This script:
# 1. Sources ROS1 Noetic
# 2. Launches AprilTag SLAM
# 3. Waits for SLAM to be ready, then starts it
# 4. Runs the ROS1 ZMQ bridge (publishes odometry to ZMQ for the ROS2 side)

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# --- Source ROS1 ---
echo -e "${BLUE}[1/4] Sourcing ROS1 Noetic...${NC}"
source /opt/ros/noetic/setup.bash
source /home/nvidia/StartUp/devel_isolated/setup.bash
echo -e "${GREEN}ROS1 sourced.${NC}"

# --- Launch SLAM in background ---
echo -e "${BLUE}[2/4] Launching AprilTag SLAM...${NC}"
roslaunch /home/nvidia/StartUp/src/AprilTagSLAM_ROS/launch/zed_sdk.launch &
SLAM_PID=$!

# --- Wait for SLAM service to be available ---
echo -e "${BLUE}[3/4] Waiting for SLAM service...${NC}"
until rosservice list 2>/dev/null | grep -q "/SLAM/Start_slam"; do
    sleep 1
    echo -n "."
    # Check if SLAM process is still alive
    if ! kill -0 $SLAM_PID 2>/dev/null; then
        echo -e "\n${RED}SLAM process died unexpectedly.${NC}"
        exit 1
    fi
done
echo ""
sleep 2  # Extra wait for service to be fully ready
rosservice call /SLAM/Start_slam
echo -e "${GREEN}SLAM started.${NC}"

# --- Run ROS1 ZMQ bridge (foreground) ---
echo -e "${BLUE}[4/4] Starting ROS1 ZMQ bridge...${NC}"
python3 ${SCRIPT_DIR}/slam_tools/odom_ros1_zmq_pub.py

# Cleanup on exit
wait $SLAM_PID
