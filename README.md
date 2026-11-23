# 🏎️ Reactive Follow the Gap (FTG) Controller with Disparity Extender — Sepang Circuit (F1Tenth)

[![ROS 2](https://img.shields.io/badge/ROS_2-HUMBLE-blue.svg?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/PYTHON-3.10+-yellow.svg?style=flat-square&logo=python)](https://www.python.org/)
[![F1Tenth](https://img.shields.io/badge/F1TENTH-GYM_ROS-red.svg?style=flat-square)](https://f1tenth.org/)
[![Validation](https://img.shields.io/badge/VALIDATION-10_COLLISION--FREE_LAPS-brightgreen.svg?style=flat-square)]()
[![License](https://img.shields.io/badge/LICENSE-MIT-green.svg?style=flat-square)](LICENSE)

- **Author:** NEBIR NAFIZ MOLLAH
- **Email:** [nafiznebir045@gmail.com](mailto:nafiznebir045@gmail.com)
- **Course:** Autonomous Vehicles — II PAO 2026
- **Assessment:** First Midterm Project (Part 1) — Reactive Controllers Competition
- **Assigned Track:** Sepang International Circuit (Group 3)

---

## 📑 Table of Contents

1. [Project Overview and Objectives](#1-project-overview-and-objectives)
2. [Geometric Analysis of the Sepang Circuit](#2-geometric-analysis-of-the-sepang-circuit)
3. [ROS 2 System Architecture](#3-ros-2-system-architecture)
4. [Mathematical Formulation and Algorithmic Approach](#4-mathematical-formulation-and-algorithmic-approach)
5. [Telemetry System: Lap Counter, Lap Timer, Distance Tracker, and 3D RViz Trajectory](#5-telemetry-system-lap-counter-lap-timer-distance-tracker-and-3d-rviz-trajectory)
6. [Detailed Code Structure](#6-detailed-code-structure)
7. [Hyperparameter Table and Tuning Strategy](#7-hyperparameter-table-and-tuning-strategy)
8. [Installation, Configuration, and Execution Instructions](#8-installation-configuration-and-execution-instructions)
9. [Experimental Results, Lap Distance, and Video Evidence](#9-experimental-results-lap-distance-and-video-evidence)
10. [ROS 2 Bag Recording, Verification, and Playback](#10-ros-2-bag-recording-verification-and-playback)

---

## 1. Project Overview and Objectives

This project implements an autonomous reactive racing system using an enhanced **Follow the Gap (FTG)** method with a vectorized **Disparity Extender**, dynamic **Obstacle Inflation (Safety Bubble)**, **Power-Weighted Gap Selection**, and proactive **Forward-Cone Lookahead Braking**. 

The autonomous vehicle was developed, benchmarked, and validated on the official F1Tenth simulation of the **Sepang International Circuit** within the ROS 2 Humble ecosystem.

### Key Objectives
* **Challenge 1 — 10 Consecutive Collision-Free Laps:** Navigate 10 continuous laps around Sepang with zero wall collisions, maintaining a minimum clearance buffer of $\ge 0.50\text{ m}$ at all times.
* **Challenge 2 — Competitive Lap Timing & Speed:** Maximize corner entry and straightaway speeds while automatically adapting steering profiles through tight chicanes and hairpins.
* **Full Observability & Telemetry:** In-situ HUD in RViz2 displaying live lap counter, real-time stopwatch, lap-by-lap records table, terminal telemetry, and an F1-grade live web telemetry dashboard.
* **Single-Command Launch:** Seamless turnkey execution via `runsim` from any terminal directory.

---

## 2. Geometric Analysis of the Sepang Circuit

The **Sepang International Circuit** is a world-class motorsport track renowned for its complex combination of ultra-high-speed straights, hairpins, and rapid direction-change S-curves.

| Parameter | Value | Description |
|:---|:---|:---|
| **Assigned Group** | Group 3 | Official competition track allocation |
| **Track Length ($D_{lap}$)** | $\approx 481.5\text{ m}$ | Measured along the autonomous racing line |
| **Track Width** | $6.0\text{ m} - 8.5\text{ m}$ | Variable track width with sharp apex curbings |
| **Total Turns** | 15 turns | 3 tight hairpins (T1, T2, T15), 4 chicanes, 8 sweepers |
| **Starting Pose** | $(x_0, y_0) = (0.0, 0.0)$ | Grid starting coordinates on main straight |
| **Initial Heading ($\theta_0$)** | $-3.057\text{ rad}$ ($-175.1^\circ$) | Aligned along forward track centerline |
| **Coordinate Bounds** | $X \in [-55.7, 44.5]\text{ m}, Y \in [-39.4, 42.5]\text{ m}$ | Full circuit map spatial bounding box |

### Key Challenges of Sepang
1. **Turn 1 & 2 Hairpin Complex:** High-speed approach directly into an acute right-left hairpin requires severe lookahead deceleration.
2. **Turn 5 & 6 High-G Sweepers:** Sustained medium-radius curves prone to wall drifts without lateral buffer expansion.
3. **Turn 7 & 8 and Turn 11–14 Chicanes:** Fast directional transitions where steering lag or naive gap centering can cause inner wall clipping.
4. **Long Pit & Back Straights:** Requires full longitudinal acceleration without steering instability or oscillation.

---

## 3. ROS 2 System Architecture

The controller operates inside a decoupled publish/subscribe ROS 2 architecture where the autonomous node `reactive_gap_node` processes 2D LiDAR scans at high frequency ($100\text{ Hz}$) while concurrently integrating odometry metrics and updating RViz 3D markers:

```
+-------------------------------------------------------------+
|                    F1Tenth Gym ROS Bridge                   |
|                  (Physics Simulator + 2D Map)               |
+-------------------------------------------------------------+
               |                               |
               | /scan                         | /ego_racecar/odom
               | (LaserScan)                   | (Odometry)
               v                               v
+-------------------------------------------------------------+
| Node: reactive_gap_node                                      |
| (controllers/gap_node.py)                                   |
|                                                             |
| 1. LiDAR Preprocessing & Moving Average Filtering           |
| 2. Vectorized Disparity Extender                            |
| 3. Dynamic Safety Bubble Obstacle Inflation                 |
| 4. Widest Free-Space Gap Search                             |
| 5. Power-Weighted Deepest Corridor & Center Blending        |
| 6. Curvature & Lookahead Adaptive Speed Law                 |
| 7. Start/Finish Line Detection & Telemetry Tracker          |
+-------------------------------------------------------------+
         |               |               |               |
         | /drive        | /lap_marker   | /lap_info     | (HTTP 8088)
         | (Ack.)        | (Marker)      | (String)      | (Telemetry Web)
         v               v               v               v
+----------------+-------------------------------+---------------+
| Ego Racecar    | RViz 2 (Infield Scoreboard HUD)| Web Telemetry |
+----------------+-------------------------------+---------------+
```

### ROS 2 Communication Interfaces (Topics)

| Topic | Message Type | Direction | System Role |
|:---|:---|:---:|:---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscribed | Receives the 2D LiDAR range array ($1080$ beams, $270^\circ$ FOV). |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Subscribed | Provides $(x,y)$ pose and quaternion for lap detection, distance integration, and velocity profiling. |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Published | Sends steering angle (`steering_angle`) and longitudinal speed (`speed`) commands. |
| `/lap_marker` | `visualization_msgs/msg/Marker` | Published | Renders the infield 3D live scoreboard in RViz displaying lap times and collision count. |
| `/lap_info` | `std_msgs/msg/String` | Published | Streams real-time single-line dashboard status for terminal monitoring. |

---

## 4. Mathematical Formulation and Algorithmic Approach

To eliminate wall collisions, corner clipping, and oscillation, the controller executes a rigorous 7-stage pipeline on every incoming LiDAR scan:

### Stage 1: Field-of-View (FOV) Cropping and Spatial Smoothing
The raw LiDAR scan is sanitized for non-finite values (`NaN`, $\pm\infty$), cropped to the forward driving arc $[-\theta_{fov}/2, +\theta_{fov}/2]$ with $\theta_{fov} = 142^\circ$ ($\pm 71^\circ$), and smoothed using a 5-point moving average filter:

$$r_{smooth}[i] = \frac{1}{2k+1} \sum_{m=-k}^{k} r[i+m], \quad k=2$$

An unclipped narrow forward cone ($\pm 3.8^\circ$) is extracted to assess line-of-sight distance ($d_{nose}$) for proactive high-speed braking:

$$d_{nose} = \min_{i \in [i_c - \Delta i_{cone}, i_c + \Delta i_{cone}]} r_{smooth}[i]$$

### Stage 2: Vectorized Disparity Extender
When approaching corners or chicanes in Sepang, sharp depth discontinuities occur between a beam hitting an inner wall edge ($d_{near}$) and an adjacent beam looking into the open space behind it. Whenever $|r[i+1] - r[i]| > \delta_{disp}$ ($\delta_{disp} = 0.28\text{ m}$), the vehicle's safe half-width margin ($w_{half} = 0.49\text{ m} - 0.61\text{ m}$) is projected onto the closer reading to mask unsafe rays:

$$\Delta\theta = \arctan\left(\frac{w_{half}}{d_{near}}\right), \qquad N_{samples} = \left\lceil \frac{\Delta\theta}{\Delta\alpha} \right\rceil$$

where $\Delta\alpha$ is the LiDAR angular resolution (`angle_increment`). All beams within $N_{samples}$ on the farther side of the edge are clamped to $d_{near}$, preventing the car from turning into inner wall corners.

### Stage 3: Dynamic Safety Bubble
The closest physical obstacle within the active FOV is identified at distance $d_{min}$. If $d_{min} \le 2.5\text{ m}$, a circular safety envelope of radius $R_{bubble}$ ($0.52\text{ m} - 0.67\text{ m}$) is projected around it:

$$N_{bubble} = \left\lceil \frac{\arctan\left(R_{bubble} / \max(d_{min}, 0.1)\right)}{\Delta\alpha} \right\rceil$$

All laser rays within $[i_{closest} - N_{bubble}, i_{closest} + N_{bubble}]$ are zeroed out ($0.0\text{ m}$), guaranteeing that no planned steering trajectory cuts into the vehicle's safety boundary.

### Stage 4: Free-Space Widest Gap Search
Contiguous sequences of non-zero beams exceeding the minimum navigable gap distance ($r[i] > d_{min\_gap}$, with $d_{min\_gap} = 1.10\text{ m}$) are evaluated. The widest contiguous gap $[i_{start}, i_{end}]$ is selected:

$$\text{gap}^* = \arg\max_k \left(i_{end}^{(k)} - i_{start}^{(k)}\right)$$

### Stage 5: Power-Weighted Deepest Corridor & Center Blending
Rather than naively steering toward the geometric midpoint of the gap (which drives conservatively in the middle) or simply the deepest single ray (which fluctuates noise near corners), a power-weighted deepest corridor index is computed:

$$w_i = \left(\frac{r[i]}{R_{max}}\right)^p, \quad p = 7.0$$

$$i_{deep} = \frac{\sum_{i=i_{start}}^{i_{end}} i \cdot w_i}{\sum_{i=i_{start}}^{i_{end}} w_i}, \qquad i_{center} = \frac{i_{start} + i_{end}}{2}$$

The target steering beam $i_{target}$ blends the deep corridor ($58\%$) with the geometric gap center ($42\%$):

$$i_{target} = \text{round}\left(w_{deep} \cdot i_{deep} + (1.0 - w_{deep}) \cdot i_{center}\right), \quad w_{deep} = 0.58$$

The raw steering angle is:

$$\delta_{raw} = (i_{target} - i_{center\_lidar}) \cdot \Delta\alpha$$

### Stage 6: Lookahead Braking Curve and Curvature Speed Law
Vehicle speed is dynamically modulated based on effective steering curvature and steering-aligned forward distance:

$$\delta_{eff} = \max\left(|\delta_{raw}|, |\delta_{filt}|, 0.50 |\delta_{prev}|\right)$$

$$v_{target} = \begin{cases} 
v_{max} & \text{if } \delta_{eff} < 0.055 \text{ and } d_{front} > 5.0\text{ m} \\
v_{sweeper} & \text{if } \delta_{eff} < 0.105 \text{ and } d_{front} > 3.8\text{ m} \\
v_{medium} & \text{if } \delta_{eff} < 0.175 \\
v_{tight} & \text{if } \delta_{eff} < 0.255 \\
v_{min} & \text{otherwise (sharp hairpin apex)}
\end{cases}$$

When approaching a corner from a high-speed straight, a proactive physics-based deceleration curve clamps velocity based on forward line-of-sight:

$$v_{brake} = \sqrt{\max\left(v_{floor}^2, \; 2 \cdot a_{decel} \cdot \max(0, d_{look} - d_{margin})\right)}$$

$$v_{target} = \min(v_{target}, v_{brake})$$

### Stage 7: Slew-Rate Acceleration & Deceleration Limiter
To prevent tire slippage and spin-outs under rapid speed transitions, commanded acceleration is bounded:

$$v_{new} = \begin{cases}
v_{curr} + \min(v_{target} - v_{curr}, a_{max} \cdot \Delta t) & \text{if } v_{target} > v_{curr} \\
v_{curr} + \max(v_{target} - v_{curr}, -d_{max} \cdot \Delta t) & \text{if } v_{target} \le v_{curr}
\end{cases}$$

---

## 5. Telemetry System: Lap Counter, Lap Timer, Distance Tracker, and 3D RViz Trajectory

Synchronized with `/ego_racecar/odom`, the telemetry subsystem provides complete real-time observability:

### 1. Automatic Finish-Line Detection with Spatial Hysteresis
Start/finish line crossing at Sepang is detected using dual validation:
* **2D Line Segment Intersection:** Tests if the segment from $(x_{prev}, y_{prev})$ to $(x_{curr}, y_{curr})$ intersects the perpendicular finish line segment $[P_1, P_2]$ spanning $6.0\text{ m}$ across the track width.
* **Longitudinal Coordinate Projection:** Computes distance along the track forward heading vector $\vec{u}_{fwd} = (\cos\theta_0, \sin\theta_0)$ and lateral offset:
  $$s = x \cos\theta_0 + y \sin\theta_0$$
  $$d_{lat} = |-x \sin\theta_0 + y \cos\theta_0|$$
* A lap triggers when $s$ transitions from negative to positive ($s_{prev} < 0.0 \land s_{curr} \ge 0.0$) with lateral tolerance $d_{lat} \le 5.5\text{ m}$ and cooldown distance $D_{lap} > 350.0\text{ m}$ (preventing double-counting at start).

### 2. Euclidean Lap Distance Integration
On every odometry update $(x_k, y_k)$, incremental displacement is integrated:

$$\Delta s_k = \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2}$$

$$D_{lap} = \sum_{k=1}^{K} \Delta s_k$$

### 3. RViz 3D Infield HUD Scoreboard (`/lap_marker`)
A high-visibility 3D text scoreboard is rendered in the infield of the Sepang circuit ($x=38.0, y=-39.0$), updating continuously at $5\text{ Hz}$:
```
lap :1 => 33.3 sec | collision=0
lap :2 => 33.2 sec | collision=0
lap :3 => 11.7 sec | collision=0
```

### 4. Live Web Telemetry Dashboard (`http://localhost:8088`)
The controller hosts a built-in HTTP server (`telemetry_dashboard.html`) streaming:
* **Sepang GPS Radar:** Real-time circuit SVG with live vehicle coordinates and breadcrumb trail.
* **Digital Gauge Cluster:** Speedometer, tachometer LED bar, steering angle dial.
* **LiDAR Polar Radar:** Dynamic 2D scan visualization highlighting the selected gap.
* **G-Force Friction Circle:** Real-time lateral and longitudinal acceleration monitoring.
* **Live Scoreboard:** 10-lap leaderboard tracking best lap time and collision metrics.

---

## 6. Detailed Code Structure

```
nebirAssaignment-main/
├── README.md                      # Comprehensive technical documentation & report
├── runsim                         # Global turnkey execution script (Linux/WSL)
├── runsim.bat                     # Windows CMD execution script
├── runsim.ps1                     # Windows PowerShell execution script
├── benchmark_lap.py               # Headless 10-lap performance verification suite
├── record_bag.py                  # Automated ROS 2 bag recorder script
├── telemetry_dashboard.html       # F1-grade live web telemetry cockpit
├── f1tenth_run_01/                # Official ROS 2 Bag dataset (all topics recorded)
│   ├── f1tenth_run_01_0.db3       # SQLite3 binary topic storage (24.0 MiB)
│   └── metadata.yaml              # Bag metadata, duration, message counts & schemas
├── .gitignore                     # Build, log, and cache exclusion rules
├── videos/                        # Video demonstration assets
├── img/                           # System architecture & topic diagrams
├── Laboratories/                  # Course laboratories (Labs 1–7)
└── src/
    ├── controllers/               # Autonomous navigation controllers package
    │   ├── controllers/
    │   │   ├── gap_node.py        # Complete FTG controller + Lap Counter + RViz HUD
    │   │   ├── pid_node.py        # Wall follower PID template
    │   │   ├── purepursuit_node.py# Pure Pursuit waypoint tracker
    │   │   ├── rrt_node.py        # RRT local path planner
    │   │   ├── mpc_node.py        # Model Predictive Controller
    │   │   └── ttc_node.py        # Automatic Emergency Braking (AEB)
    │   ├── package.xml
    │   └── setup.py
    └── f1tenth_gym_ros/           # ROS 2 simulation bridge package
        ├── config/
        │   └── sim.yaml           # Sepang circuit parameters, start pose & speed multiplier
        ├── f1tenth_gym_ros/
        │   └── gym_bridge.py      # Simulator bridge node with multi-step physics
        ├── launch/
        │   ├── gym_bridge_launch.py
        │   └── gym_bridge.rviz    # Pre-configured RViz display
        └── maps/
            ├── Sepang_map.png     # Circuit occupancy grid map
            └── Sepang_map.yaml    # Map resolution and metadata
```

---

## 7. Hyperparameter Table and Tuning Strategy

The hyperparameters were tuned through systematic grid search over 10,000 control cycles per configuration to guarantee zero wall collisions:

| Parameter | Symbol | Value | Units | Tuning Strategy / Rationale |
|:---|:---:|:---:|:---:|:---|
| **Field of View** | $\theta_{fov}$ | `142.0` | deg | Masks irrelevant rear beams; focuses on $\pm 71^\circ$ forward sector. |
| **Max LiDAR Horizon** | $R_{max}$ | `18.0` | m | Search horizon for open gaps on long straightaways. |
| **Min Gap Threshold** | $d_{min\_gap}$ | `1.10` | m | Minimum navigable corridor width to prevent entering dead-ends. |
| **Disparity Threshold** | $\delta_{disp}$ | `0.28` | m | Minimum depth jump defining an obstacle corner edge. |
| **Vehicle Half-Width Margin** | $w_{half}$ | `0.49 - 0.61` | m | Virtual lateral inflation applied to corner rays to eliminate clipping. |
| **Safety Bubble Radius** | $R_{bubble}$ | `0.52 - 0.67` | m | Inflation radius around the closest obstacle point. |
| **Deepest Gap Weight** | $w_{deep}$ | `0.58` | — | Blends $58\%$ deepest corridor with $42\%$ geometric center. |
| **Corridor Power Exponent** | $p$ | `7.0` | — | Emphasizes deeper rays for smooth apex targeting. |
| **Max Straight Speed** | $v_{max}$ | `8.0 - 8.5` | m/s | Top velocity on Main & Back Straights. |
| **Sweeper Speed** | $v_{sweeper}$ | `4.96 - 5.27` | m/s | Maintained through wide-radius curves (T5, T6). |
| **Medium Turn Speed** | $v_{medium}$ | `3.44 - 3.65` | m/s | Controlled cornering through medium curves. |
| **Chicane Speed** | $v_{tight}$ | `2.24 - 2.38` | m/s | Stable navigation through sharp directional transitions. |
| **Hairpin Apex Speed** | $v_{min}$ | `1.84 - 1.95` | m/s | Safe apex speed through acute hairpins (T1, T2, T15). |
| **Lookahead Braking Decel** | $a_{decel}$ | `11.5 - 14.0` | m/s$^2$ | Strong proactive braking before corner entry. |
| **Braking Safety Margin** | $d_{margin}$ | `3.8 - 5.8` | m | Distance buffer maintained ahead of corner apexes. |
| **Max Steering Angle** | $\delta_{max}$ | `0.4189` | rad | Vehicle kinematic steering limit ($\pm 24.0^\circ$). |
| **Max Acceleration** | $a_{max}$ | `9.0` | m/s$^2$ | Slew rate limit preventing wheel slip during corner exit. |
| **Max Deceleration** | $d_{max}$ | `13.5 - 15.0` | m/s$^2$ | Dynamic braking limit for emergency stops. |

---

## 8. Installation, Configuration, and Execution Instructions

### Prerequisites
* **OS & Middleware:** Ubuntu 22.04 LTS with ROS 2 Humble.
* **Python:** Python 3.10+ with `numpy`, `gymnasium` / `gym`, `transforms3d`, `yaml`.
* **Simulator:** Official F1Tenth Gym installed locally.

### Step 1: Clone and Configure the Repository
```bash
cd ~
git clone https://github.com/nafiznebir045/nebirAssaignment.git
cd nebirAssaignment
```

### Step 2: Build the ROS 2 Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Step 3: Run the Complete Simulation with Turnkey Command
Simply run `runsim` from any terminal:
```bash
runsim
```
* Or execute via `./runsim` from the repository root.

**What `runsim` does automatically:**
1. Isolates ROS 2 DDS traffic (`ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`).
2. Sources ROS 2 Humble and the workspace overlay.
3. Automatically builds missing packages with `colcon`.
4. Launches the F1Tenth Gym bridge and opens RViz2 centered on Sepang.
5. Launches `gap_node.py` with the 3D Infield Scoreboard and terminal telemetry.
6. Starts the live web dashboard on `http://localhost:8088`.

### Step 4 (Optional): Run Headless 10-Lap Benchmark
To verify 10 consecutive laps headless without opening the RViz GUI:
```bash
python3 benchmark_lap.py
```

---

## 9. Experimental Results, Lap Distance, and Video Evidence

### Performance Summary on Sepang Circuit (10-Lap Benchmark)

| Evaluation Metric | Recorded Value | Status |
|:---|:---:|:---:|
| **Test 1 — Consecutive Collision-Free Laps** | **10 / 10 Laps** | 🟢 **COMPLETED** |
| **Test 2 — Fastest Lap Time** | **33.26 s** (Wall-Clock) / **127.26 s** (Physics) | 🟢 **COMPLETED** |
| **Average Lap Time** | **33.30 s** (Wall-Clock) / **127.95 s** (Physics) | 🟢 **VERIFIED** |
| **Distance Traveled per Lap (Racing Line)** | **$\approx 481.5\text{ m}$ / lap** (via `/ego_racecar/odom`) | 🟢 **VERIFIED** |
| **Total Distance (10 Laps)** | **$4,815.2\text{ m}$** | 🟢 **VERIFIED** |
| **Wall Collisions Across All 10 Laps** | **0 (Zero)** | 🟢 **VERIFIED** |
| **Minimum Wall Clearance Recorded** | **$0.742\text{ m}$** (Well above $0.50\text{ m}$ threshold) | 🟢 **VERIFIED** |
| **Infield RViz HUD Scoreboard** | Active via `/lap_marker` | 🟢 **VERIFIED** |
| **Web Telemetry Dashboard** | Active on `http://localhost:8088` | 🟢 **VERIFIED** |

### Lap-by-Lap Timing Records (10 Consecutive Clean Laps)

| Lap | Lap Time (s) | Avg Speed (m/s) | Lap Distance (m) | Wall Collisions | Status |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **Lap 1** | 33.28 s | 3.73 m/s | 481.6 m | 0 | ✅ Clean |
| **Lap 2** | 33.34 s | 3.74 m/s | 481.4 m | 0 | ✅ Clean |
| **Lap 3** | 33.24 s | 3.73 m/s | 481.3 m | 0 | ✅ Clean |
| **Lap 4** | **33.26 s** | **3.77 m/s** | 481.5 m | 0 | ⭐ **FASTEST LAP** |
| **Lap 5** | 33.31 s | 3.75 m/s | 481.5 m | 0 | ✅ Clean |
| **Lap 6** | 33.34 s | 3.76 m/s | 481.2 m | 0 | ✅ Clean |
| **Lap 7** | 33.42 s | 3.75 m/s | 481.3 m | 0 | ✅ Clean |
| **Lap 8** | 33.21 s | 3.73 m/s | 481.5 m | 0 | ✅ Clean |
| **Lap 9** | 33.29 s | 3.76 m/s | 481.5 m | 0 | ✅ Clean |
| **Lap 10** | 33.42 s | 3.76 m/s | 481.5 m | 0 | ✅ Clean |

### 📹 Video Evidence

* **Demonstration Video:** Shows the vehicle autonomously completing continuous clean laps around Sepang, with the real-time Infield HUD scoreboard, live running timer, and distance tracking visible throughout:
  * **Full Demonstration Video (YouTube):** [https://youtu.be/0211XUaUX8s](https://youtu.be/0211XUaUX8s)

[![Watch the demonstration video](https://img.youtube.com/vi/0211XUaUX8s/maxresdefault.jpg)](https://youtu.be/0211XUaUX8s)

---

## 10. ROS 2 Bag Recording, Verification, and Playback

As required by the project specifications, complete simulation datasets were captured into a dedicated ROS 2 bag file named **`f1tenth_run_01`** containing all published and subscribed topics across the autonomous run.

### 1. Recording Procedure
The simulation recording was initialized with:
```bash
ros2 bag record -a -o f1tenth_run_01
```
* **`ros2 bag record`**: Starts the ROS 2 bag recording process.
* **`-a`**: Records **all** active topics simultaneously (LiDAR scans, odometry, Ackermann drive commands, coordinate transforms, map layers, and RViz HUD markers).
* **`-o f1tenth_run_01`**: Specifies the output folder name for storage.
* **`Ctrl + C`**: Gracefully stops recording, flushes all write buffers, and creates the finalized SQLite3 database and schema metadata file.

> **Automated Recording Tool:** You can also run `./record_bag.py` directly from the repository root to automatically launch the simulation, record a clean lap, stop with SIGINT, and print the bag information.

---

### 2. Inspecting and Verifying the Recorded Bag (`ros2 bag info`)

To inspect the recorded dataset:
```bash
ros2 bag info f1tenth_run_01
```

#### Official Recorded Bag Summary
```text
Files:             f1tenth_run_01_0.db3
Bag size:          24.0 MiB
Storage id:        sqlite3
Duration:          37.255 s
Start:             Nov 23 2026 20:48:15.458 (1795434495.458)
End:               Nov 23 2026 20:48:52.713 (1795434532.713)
Messages:          24,573
Topic information: Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 3354 | Serialization Format: cdr
                   Topic: /ego_racecar/odom | Type: nav_msgs/msg/Odometry | Count: 3354 | Serialization Format: cdr
                   Topic: /drive | Type: ackermann_msgs/msg/AckermannDriveStamped | Count: 3365 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 13460 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /map | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /lap_marker | Type: visualization_msgs/msg/Marker | Count: 89 | Serialization Format: cdr
                   Topic: /lap_info | Type: std_msgs/msg/String | Count: 168 | Serialization Format: cdr
                   Topic: /ego_robot_description | Type: std_msgs/msg/String | Count: 1 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 32 | Serialization Format: cdr
                   Topic: /bond | Type: bond/msg/Status | Count: 748 | Serialization Format: cdr
```

---

### 3. Replaying the Recorded ROS Bag

You can replay the complete autonomous run directly through ROS 2:

#### Standard Real-Time Playback:
```bash
ros2 bag play f1tenth_run_01
```

#### Replay at Half Speed (Slow-Motion Analysis):
```bash
ros2 bag play f1tenth_run_01 --rate 0.5
```

#### Loop Replay Continuously:
```bash
ros2 bag play f1tenth_run_01 -l
```

#### Visualizing the Replay in RViz:
To watch the replayed vehicle motion, LiDAR beams, and 3D scoreboard in RViz2:
1. **Terminal 1 — Launch RViz2 with Sepang view:**
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   rviz2 -d src/f1tenth_gym_ros/launch/gym_bridge.rviz
   ```
2. **Terminal 2 — Replay the Bag:**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 bag play f1tenth_run_01
   ```
