#!/usr/bin/env bash
# ==============================================================================
# F1TENTH Real Clock RViz Live Runner
# Driver: NEBIR NAFIZ MOLLAH | Group 3 | Sepang International Circuit
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

cleanup() {
    echo -e "\n\033[1;33m[!] Shutting down RViz and simulation nodes...\033[0m"
    pkill -9 -x gym_bridge 2>/dev/null || true
    pkill -9 -x gap_node 2>/dev/null || true
    pkill -9 -x rviz2 2>/dev/null || true
    pkill -9 -x robot_state_publisher 2>/dev/null || true
    pkill -9 -x map_server 2>/dev/null || true
    pkill -9 -x lifecycle_manager 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM SIGHUP

pkill -9 -x gym_bridge 2>/dev/null || true
pkill -9 -x gap_node 2>/dev/null || true
pkill -9 -x rviz2 2>/dev/null || true
pkill -9 -x robot_state_publisher 2>/dev/null || true
sleep 0.5

export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42

source /opt/ros/humble/setup.bash
source /home/admin1/ros2_ws/install/setup.bash

echo -e "\033[1;36m========================================================================\033[0m"
echo -e "\033[1;32m🏎️   F1TENTH LIVE RVIZ 3D SIMULATION - REAL CLOCK TIME\033[0m"
echo -e "\033[1;37m    Driver : NEBIR NAFIZ MOLLAH | Group 3\033[0m"
echo -e "\033[1;37m    Track  : Sepang International Circuit (with 5 Static Obstacles)\033[0m"
echo -e "\033[1;37m    Window : Opening RViz 3D on your screen (DISPLAY=:0)...\033[0m"
echo -e "\033[1;36m========================================================================\033[0m\n"

# Copy latest configurations
cp -r "$SCRIPT_DIR/src/f1tenth_gym_ros/"* /home/admin1/ros2_ws/src/F1Tenth-Repository/src/f1tenth_gym_ros/
cp -r "$SCRIPT_DIR/src/controllers/"* /home/admin1/ros2_ws/src/F1Tenth-Repository/src/controllers/

# Launch simulator bridge and RViz
ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
SIM_PID=$!
sleep 3.5

# Launch autonomous controller
ros2 run controllers gap_node
wait $SIM_PID
