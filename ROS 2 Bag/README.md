# 📦 ROS 2 Bag Dataset — F1TENTH Autonomous Racing

[![ROS 2](https://img.shields.io/badge/ROS_2-HUMBLE-blue.svg?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Format](https://img.shields.io/badge/FORMAT-SQLITE3-lightgrey.svg?style=flat-square)](https://www.sqlite.org/)
[![Size](https://img.shields.io/badge/SIZE-24.0_MiB-brightgreen.svg?style=flat-square)]()
[![Validation](https://img.shields.io/badge/STATUS-VERIFIED_CLEAN_RUN-success.svg?style=flat-square)]()

- **Author / Driver:** NEBIR NAFIZ MOLLAH
- **Assigned Track:** Sepang International Circuit (Group 3)
- **Course:** Autonomous Vehicles — II PAO 2026
- **Storage Directory:** `ROS 2 Bag/`

---

## 📋 Dataset Summary

This folder contains the complete, verified ROS 2 bag recorded during autonomous navigation of the F1TENTH racecar on the **Sepang International Circuit**. All primary vehicle topics—including 2D LiDAR scans, odometry, Ackermann drive commands, coordinate transforms, map layers, and 3D HUD markers—were synchronously recorded.

| Attribute | Value |
|:---|:---|
| **Bag Directory** | `ROS 2 Bag/` |
| **Storage Backend** | SQLite3 (`f1tenth_run_01_0.db3`) |
| **Metadata File** | `metadata.yaml` |
| **Duration** | $37.26\text{ seconds}$ |
| **Total Message Count** | $24,573\text{ messages}$ |
| **Bag Size** | $24.0\text{ MiB}$ |
| **Compression** | None (Raw CDR serialization) |
| **Telemetry CSV Logs** | `lap_times.csv`, `lap_times_static_obstacles.csv`, `lap_times_dynamic_overtaking.csv` |

---

## 📊 Recorded Topics Breakdown

| Topic | Message Type | Message Count | Frequency | Description |
|:---|:---|:---:|:---:|:---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 3,354 | ~100 Hz | 2D LiDAR range arrays (1080 beams, 270° FOV) |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | 3,354 | ~100 Hz | Vehicle $(x,y)$ position, velocity & orientation |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | 3,365 | ~100 Hz | Reactive steering angle & speed commands |
| `/tf` | `tf2_msgs/msg/TFMessage` | 13,460 | ~400 Hz | Dynamic coordinate frame transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 | Latched | Static robot link transforms |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 1 | Latched | High-resolution Sepang circuit map layer |
| `/lap_marker` | `visualization_msgs/msg/Marker` | 89 | ~5 Hz | 3D RViz Infield HUD Scoreboard |
| `/lap_info` | `std_msgs/msg/String` | 168 | ~5 Hz | Terminal telemetry text stream |
| `/ego_robot_description` | `std_msgs/msg/String` | 1 | Latched | URDF vehicle visual description |
| `/bond` | `bond/msg/Status` | 748 | ~20 Hz | Lifecycle manager bond status |
| `/rosout` | `rcl_interfaces/msg/Log` | 32 | Event-driven | ROS 2 logging channel |

---

## 🚀 Quick Playback Instructions

From the root of the repository, you can replay or inspect this bag using either the provided helper script or native ROS 2 commands:

### Method 1: Using the Turnkey Helper Script (Easiest)
```bash
# Replay in real-time:
./play_bag.sh

# Replay at half speed (slow-motion analysis):
./play_bag.sh --rate 0.5

# Inspect dataset metadata:
./play_bag.sh --info
```

### Method 2: Using Native ROS 2 CLI Commands
```bash
# Sourcing ROS 2
source /opt/ros/humble/setup.bash

# 1. Inspect the bag information:
ros2 bag info "ROS 2 Bag"

# 2. Normal playback:
ros2 bag play "ROS 2 Bag"

# 3. Half-speed playback:
ros2 bag play "ROS 2 Bag" --rate 0.5

# 4. Continuous loop playback:
ros2 bag play "ROS 2 Bag" -l
```

---

## 🖥️ Visualizing the Playback in RViz2

To watch the recorded autonomous run with full visual feedback in RViz:

1. **Terminal 1 — Launch RViz2 (Pre-configured Sepang View):**
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   rviz2 -d src/f1tenth_gym_ros/launch/gym_bridge.rviz
   ```

2. **Terminal 2 — Play the Bag:**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 bag play "ROS 2 Bag"
   ```
