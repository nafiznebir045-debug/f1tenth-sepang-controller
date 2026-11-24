#!/usr/bin/env bash
set -e
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Clean any existing processes
pkill -f "gym_bridge" 2>/dev/null || true
pkill -f "gap_node" 2>/dev/null || true
pkill -f "opp_node" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "map_server" 2>/dev/null || true
pkill -f "lifecycle_manager" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

echo "[*] Launching F1TENTH Simulator bridge with RViz..."
ros2 launch f1tenth_gym_ros gym_bridge_launch.py > /tmp/f1tenth_sim.log 2>&1 &
SIM_PID=$!

sleep 4

echo "[*] Launching Follow-the-Gap Controller (NEBIR NAFIZ MOLLAH)..."
ros2 run controllers gap_node
