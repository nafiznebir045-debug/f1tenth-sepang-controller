# Individual Oral Project Defense Preparation Guide

**Student Name:** NEBIR NAFIZ MOLLAH  
**Course:** Autonomous Vehicles  
**Instructor:** Nabih Pico, Ph.D.  
**TA:** Winter Delgado  
**Group:** Group 3 | Track: Sepang International Circuit  
**Topic:** Reactive Controller Competition in F1Tenth (Follow-the-Gap, Obstacle Avoidance, Overtaking)

---

## 📌 Defense Overview & Objective

The individual oral defense is a mandatory stage with a **multiplicative weighting factor ($\times 1.0$ vs $< 1.0$)**.  
The instructor evaluates:
1. **Authenticity:** Did you implement and understand the code yourself?
2. **Conceptual Depth:** Do you understand the mathematical mechanics of Follow-the-Gap, disparity extenders, safety bubbles, and Ackermann vehicle dynamics?
3. **Engineering Decisions:** Why did you choose specific parameters (bubble radius, disparity threshold, speed lookahead)?
4. **Competition Execution:** How did your controller achieve 10 clean laps with 5 static obstacles and execute dynamic overtaking?

---

## ❓ Technical & Conceptual Questions with Model Answers

### Q1: What is the core principle of the Follow-the-Gap (FTG) algorithm?
**Answer:**  
Follow-the-Gap is a purely reactive local motion planning algorithm. Rather than depending on a global map and heavy trajectory optimization, FTG operates directly on real-time LiDAR scan readings at high control frequency ($40\text{--}100\text{ Hz}$).  
The steps are:
1. **Perception & Filtering:** Ingest 1080 laser beams, clean NaNs/infinites, and apply a moving average filter.
2. **Safety Bubble Generation:** Find the nearest obstacle point and project an artificial zero-range safety bubble around it based on the vehicle width.
3. **Disparity Extension:** Detect depth discontinuities (edges/corners) and extend closer obstacles laterally so the car's body does not clip corners.
4. **Gap Identification:** Find all contiguous angular sectors where clearance exceeds $d_{\text{min\_gap}}$.
5. **Target Selection:** Find the widest gap and pick an optimal target point (blending deepest corridor and geometric center).
6. **Actuation:** Compute steering angle proportional to target ray angle and command velocity via `AckermannDriveStamped`.

---

### Q2: Why is the Disparity Extender necessary, and how does it prevent wall/obstacle collisions?
**Answer:**  
In 2D LiDAR scans, obstacles and track corners create sharp distance jumps (disparities) where one beam hits an obstacle at $2\text{ m}$ and the adjacent beam shoots past into open track at $15\text{ m}$.  
If an algorithm blindly aims into the open space right at the edge, the center of the car might clear the corner, but the front-outer chassis or wheels (which have a non-zero half-width, $w \approx 0.155\text{ m}$) will physically clip the wall.  
The **Disparity Extender** detects $|\Delta r| > 0.28\text{ m}$ and mathematically expands the nearer obstacle by the vehicle half-width margin ($w_{\text{margin}} = 0.40\text{ m}$):
$$\Delta \theta = \arctan\left(\frac{w_{\text{margin}}}{r_{\text{close}}}\right), \quad n_{\text{beams}} = \frac{\Delta \theta}{\Delta \theta_{\text{inc}}}$$
It sets those $n_{\text{beams}}$ of the far scan equal to $r_{\text{close}}$, mathematically closing off the hazard zone before the car can steer towards it.

---

### Q3: How did you select the Safety Bubble radius ($r_{\text{bubble}}$)? What happens if it is too large or too small?
**Answer:**  
* **If $r_{\text{bubble}}$ is too small ($< 0.25\text{ m}$):** The car will not leave enough buffer around obstacles. Sensor noise, steering latency, and vehicle inertia could cause physical contact when cornering.
* **If $r_{\text{bubble}}$ is too large ($> 0.80\text{ m}$):** On a narrow racetrack like Sepang (corridor width $\approx 2.2\text{ m}$), an oversized bubble combined with disparity extension will zero out the entire navigable corridor, leaving no valid gap ($G = \emptyset$) and forcing the car to default to center index and crash into the obstacle.
* **Optimal Selection ($0.45\text{--}0.50\text{ m}$):** Provides a robust buffer exceeding the vehicle's diagonal footprint ($\approx 0.35\text{ m}$) while ensuring at least $1.4\text{ m}$ of open traversable gap on the opposite side of the track.

---

### Q4: How did you implement and place the 5 static obstacles for Challenge 1?
**Answer:**  
Adhering to the ESPOL Class 8 lecture (`AV - Class 8.pptx`), obstacles in F1Tenth ROS 2 are parsed by `nav2_map_server` from the map image.
1. We identified 5 strategic straightaway segments across the $486.5\text{ m}$ Sepang circuit (distances $25\text{ m}$, $125\text{ m}$, $215\text{ m}$, $310\text{ m}$, and $465\text{ m}$) where the car has over $20\text{ m}$ of clear forward vision.
2. In each segment, a circular obstacle (diameter $\approx 0.25\text{ m}$) was painted with pixel value $0$ (occupied black) offset by $0.45\text{ m}$ towards either the left or right wall.
3. This leaves a generous $1.45\text{ m}$ wide free corridor on the opposite side, creating a realistic slalom obstacle course that the reactive controller navigates with zero collisions across all 10 consecutive laps.

---

### Q5: How does the Dynamic Obstacle (Overtaking) challenge work?
**Answer:**  
* The simulation spawns 2 vehicles: `ego_racecar` and `opp_racecar`.
* Ego starts at $(0, 0, -3.057\text{ rad})$ and opponent starts $5\text{ m}$ ahead at $(-4.98, -0.42, -3.057\text{ rad})$.
* Opponent runs a steady reactive controller at $2.0\text{ m/s}$. Ego runs a faster tuned controller ($4.5\text{--}6.0\text{ m/s}$).
* As ego closes the gap, its forward LiDAR rays hit the opponent vehicle. The safety bubble and disparity extender identify the opponent as a moving obstacle and bifurcate the forward fairway.
* Ego selects the wider open lane, smoothly steers around the opponent, accelerates along the straight, and overtakes the opponent without contact. Both cars then continue for 10 laps safely.

---

### Q6: How does the Lap Counter & Timer work without relying on external packages?
**Answer:**  
In `gap_node.py`, the odometry callback listens to `/ego_racecar/odom`:
1. The start/finish line is defined as a line segment at $(0, 0)$ perpendicular to the track heading ($\theta = -3.05692\text{ rad}$).
2. Each step, it tracks vehicle coordinates $(x_{k-1}, y_{k-1}) \to (x_k, y_k)$ and evaluates 2D segment intersection using counter-clockwise orientation cross-products ($\text{CCW}$).
3. A distance hysteresis filter ($d_{\text{total}} > 350\text{ m}$) ensures that initial start maneuvers or backward jitter do not cause false lap triggers.
4. When a crossing occurs, the monotonic clock calculates elapsed time $T_{\text{lap}}$, logs to terminal and CSV, updates the fastest lap, and commands smooth vehicle stop upon reaching Lap 10.

---

### Q7: What ROS 2 topics are recorded in your ROS 2 Bag (`ROS 2 Bag` / `f1tenth_run_01`)?
**Answer:**  
The bag was recorded using:
`ros2 bag record -a -o "ROS 2 Bag"`  
Verified with `ros2 bag info "ROS 2 Bag"`:
* `/scan` : 3,354 LaserScan messages (270° environment range readings)
* `/drive` : 3,365 AckermannDriveStamped messages (velocity & steering commands)
* `/ego_racecar/odom` : 3,354 Odometry messages (poses, velocity, orientation)
* `/tf` & `/tf_static` : 13,461 transformation frames between `map`, `odom`, `base_link`, `laser`
* `/lap_info` & `/lap_marker` : Telemetry markers and strings for RViz HUD display
* `/map` : OccupancyGrid representation of Sepang circuit.

---

## 🎯 Key Takeaways for Defense Day
* Remember your student name: **NEBIR NAFIZ MOLLAH**
* Remember your track: **Sepang International Circuit (Group 3)**
* Remember your lap records: **10/10 clean laps, 0 collisions, fastest lap ~188s (at 1x real-time)**
* Emphasize the **physical intuition**: FTG is reactive, safety bubble prevents driving into obstacles, disparity extender prevents chassis corner clipping, and adaptive speed profiling allows high straightaway speed while guaranteeing stable cornering.
