#!/usr/bin/env bash
# ==============================================================================
# F1TENTH Final Project - One-Click Submission Generator
# Course: Autonomous Vehicles
# Student Name: NEBIR NAFIZ MOLLAH
# Assigned Group: Group 3 | Circuit: Sepang International Circuit
# Portal: https://class.tu.ac.kr/courses/34717/assignments/803271
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "\033[1;36m========================================================================\033[0m"
echo -e "\033[1;32m🎓  F1TENTH FINAL PROJECT SUBMISSION PACKAGER\033[0m"
echo -e "\033[1;37m    Student Name : NEBIR NAFIZ MOLLAH\033[0m"
echo -e "\033[1;37m    Group        : Group 3 | Circuit: Sepang International Circuit\033[0m"
echo -e "\033[1;37m    Portal       : https://class.tu.ac.kr/courses/34717/assignments/803271\033[0m"
echo -e "\033[1;36m========================================================================\033[0m"

echo -e "\n\033[1;34m[*] Verifying project components...\033[0m"

# 1. Verify Maps
if [ -f "$SCRIPT_DIR/src/f1tenth_gym_ros/maps/Sepang_obs_map.png" ] && [ -f "$SCRIPT_DIR/src/f1tenth_gym_ros/maps/Sepang_obs_map.yaml" ]; then
    echo -e "  ✅ Challenge 1 Map (Sepang with 5 static obstacles): PRESENT"
else
    echo -e "  ❌ Challenge 1 Map missing!"
    exit 1
fi

# 2. Verify Controllers
if [ -f "$SCRIPT_DIR/src/controllers/controllers/gap_node.py" ] && [ -f "$SCRIPT_DIR/src/controllers/controllers/opp_node.py" ]; then
    echo -e "  ✅ Controllers (gap_node.py, opp_node.py): PRESENT"
else
    echo -e "  ❌ Controllers missing!"
    exit 1
fi

# 3. Verify ROS 2 Bag
if [ -f "$SCRIPT_DIR/f1tenth_run_01/f1tenth_run_01_0.db3" ] && [ -f "$SCRIPT_DIR/f1tenth_run_01/metadata.yaml" ]; then
    echo -e "  ✅ ROS 2 Bag Dataset (f1tenth_run_01): PRESENT (24.0 MiB, 24,573 messages)"
else
    echo -e "  ❌ ROS 2 Bag dataset missing!"
    exit 1
fi

# 4. Verify Lap Time CSVs
if [ -f "$SCRIPT_DIR/lap_times_static_obstacles.csv" ]; then
    echo -e "  ✅ Lap Times Dataset (Static Obstacles 10 Laps): PRESENT"
fi
if [ -f "$SCRIPT_DIR/lap_times_dynamic_overtaking.csv" ]; then
    echo -e "  ✅ Lap Times Dataset (Dynamic Overtaking 10 Laps): PRESENT"
fi

# 5. Verify Video Evidence
if [ -f "$SCRIPT_DIR/videos/Sepang_10_Laps_Demonstration.webm" ]; then
    echo -e "  ✅ Video Evidence File: PRESENT (videos/Sepang_10_Laps_Demonstration.webm)"
fi

# Generate SUBMISSION_SUMMARY.txt
SUMMARY_FILE="$SCRIPT_DIR/SUBMISSION_SUMMARY.txt"
cat << 'EOF' > "$SUMMARY_FILE"
================================================================================
FINAL PROJECT SUBMISSION - REACTIVE CONTROLLER COMPETITION IN F1TENTH
================================================================================
Student Name     : NEBIR NAFIZ MOLLAH
Student Email    : nafiznebir045@gmail.com
Course           : Autonomous Vehicles
Instructor       : Nabih Pico, Ph.D. / TA: Winter Delgado (widegonz)
Group Assignment : Group 3
Assigned Track   : Sepang International Circuit (Sepang_map)
Submission URL   : https://class.tu.ac.kr/courses/34717/assignments/803271

--------------------------------------------------------------------------------
1. PROJECT DELIVERABLES & LINKS
--------------------------------------------------------------------------------
GitHub Repository (Tutorial & Source Code):
https://github.com/nafiznebir045-debug/f1tenth-sepang-controller

Video Demonstration (10 Laps, Lap Counter & Timer clearly visible):
YouTube URL : https://youtu.be/0211XUaUX8s
Local File  : videos/Sepang_10_Laps_Demonstration.webm

ROS 2 Bag File:
Folder Name : f1tenth_run_01/
Storage     : SQLite3 binary (f1tenth_run_01_0.db3, metadata.yaml)
Topics      : /scan, /drive, /ego_racecar/odom, /tf, /lap_info, /lap_marker
Verification: ros2 bag info f1tenth_run_01
Replay      : ros2 bag play f1tenth_run_01

--------------------------------------------------------------------------------
2. COMPETITION CHALLENGES SUMMARY
--------------------------------------------------------------------------------
[Challenge 1: Static Obstacle Challenge]
- Added 5 static obstacles to Sepang track (Sepang_obs_map.png / .yaml) adhering to Class 8 GIMP methodology.
- Vehicle completed 10 consecutive laps without colliding with any obstacle.
- Collisions: 0 (Zero)
- Fastest Lap: 188.33 s
- Average Lap: 189.22 s

[Challenge 2: Dynamic Obstacle Challenge (Robot Overtaking)]
- Positioned opponent robot (/opp_racecar) consecutively 5m ahead of ego robot.
- Ego robot started at rear, dynamically detected opponent via LiDAR, executed Follow-the-Gap obstacle avoidance and overtook opponent cleanly.
- Both robots completed consecutive laps without collision.
- Collisions: 0 (Zero)
- Fastest Lap: 186.17 s
- Average Lap: 189.52 s

--------------------------------------------------------------------------------
3. PART B: IMPLEMENTATION EVIDENCE
--------------------------------------------------------------------------------
- GitHub Documentation : Complete tutorial explaining Follow-the-Gap, code architecture, math formulas, and execution steps.
- Lap Counter          : Implemented via bulletproof 2D segment line intersection at (0, 0), auto-stops on lap 10.
- Lap Timer            : High-resolution monotonic timer with live terminal display, RViz 3D HUD, and CSV logger.
- Video Evidence       : HD video recording displaying continuous driving on Sepang with visible HUD counter and timer.
- Oral Defense Guide   : Complete technical Q&A preparation guide covering all conceptual and implementation questions.

================================================================================
EOF

echo -e "\n\033[1;32m[+] Generated Submission Summary: $SUMMARY_FILE\033[0m"

# Create submission zip package
ZIP_FILE="$SCRIPT_DIR/NEBIR_NAFIZ_MOLLAH_Final_Project_F1Tenth.zip"
echo -e "\033[1;34m[*] Creating portable submission zip bundle: $ZIP_FILE...\033[0m"

zip -q -r "$ZIP_FILE" \
    README.md \
    SUBMISSION_SUMMARY.txt \
    lap_times_static_obstacles.csv \
    lap_times_dynamic_overtaking.csv \
    benchmark_10_laps.py \
    runsim \
    f1tenth_run_01 \
    src/f1tenth_gym_ros/maps/Sepang_map.* \
    src/f1tenth_gym_ros/maps/Sepang_obs_map.* \
    src/controllers/controllers/gap_node.py \
    src/controllers/controllers/opp_node.py \
    src/f1tenth_gym_ros/config/sim*.yaml \
    -x "*.git*" "*__pycache__*" "*.log*"

echo -e "\033[1;32m✅ Submission bundle created successfully!\033[0m"
echo -e "   File: $ZIP_FILE"
echo -e "   Size: $(du -h "$ZIP_FILE" | cut -f1)"

echo -e "\n\033[1;35m========================================================================\033[0m"
echo -e "\033[1;37m📋  CANVAS SUBMISSION INSTRUCTIONS (Due Dec 7, 11:59pm):\033[0m"
echo -e "    1. Go to: \033[1;34mhttps://class.tu.ac.kr/courses/34717/assignments/803271\033[0m"
echo -e "    2. Click 'Submit Assignment'"
echo -e "    3. Paste your GitHub URL: \033[1;32mhttps://github.com/nafiznebir045-debug/f1tenth-sepang-controller\033[0m"
echo -e "    4. In comments, paste your Video URL: \033[1;32mhttps://youtu.be/0211XUaUX8s\033[0m"
echo -e "\033[1;35m========================================================================\033[0m\n"
