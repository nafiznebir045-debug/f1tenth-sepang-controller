#!/usr/bin/env bash
# ==============================================================================
# F1TENTH ROS 2 Bag Replay Utility
# Driver: NEBIR NAFIZ MOLLAH | Circuit: Sepang (Group 3)
# ==============================================================================

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="$SCRIPT_DIR/ROS 2 Bag"

if [ ! -d "$BAG_DIR" ]; then
    echo "Error: Directory '$BAG_DIR' not found!"
    exit 1
fi

echo "=============================================================="
echo "🏎️  F1TENTH ROS 2 BAG PLAYBACK — SEPANG CIRCUIT"
echo "📁  Dataset Directory : $BAG_DIR"
echo "=============================================================="

# Inspect if requested with --info
if [ "$1" == "--info" ] || [ "$1" == "-i" ]; then
    ros2 bag info "$BAG_DIR"
    exit 0
fi

echo -e "\n[*] Starting playback (Press Ctrl+C to stop)..."
echo -e "[*] Tip: To play at half speed, run: ./play_bag.sh --rate 0.5\n"

ros2 bag play "$BAG_DIR" "$@"
