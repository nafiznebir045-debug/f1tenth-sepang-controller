#!/usr/bin/env python3
"""
==============================================================================
F1TENTH Autonomous Racing - Live 10-Lap Visual Simulation & Scoreboard GUI
Course: Autonomous Vehicles | Final Project
Driver: NEBIR NAFIZ MOLLAH
Group: 3 | Track: Sepang International Circuit
==============================================================================
Renders a live, real-time 60 FPS graphical window showing the F1TENTH car
navigating 10 consecutive laps around Sepang with 5 static obstacles,
featuring an on-screen HUD, live lap counter, running lap timer, and
infield scoreboard with 0 collisions.
"""

import os
import sys
import time
import math
import cv2
import numpy as np

try:
    import gymnasium as gym
except ImportError:
    import gym
import f110_gym

# Import FollowTheGapController from benchmark_10_laps
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)
from benchmark_10_laps import FollowTheGapController, intersect


def run_live_gui(target_laps=10, speed_multiplier=4):
    map_path = os.path.join(script_dir, 'src/f1tenth_gym_ros/maps/Sepang_obs_map')
    map_png = map_path + '.png'

    # Load base map image
    base_gray = cv2.imread(map_png, cv2.IMREAD_GRAYSCALE)
    if base_gray is None:
        print(f"Error: Map image not found at {map_png}")
        return False

    h_map, w_map = base_gray.shape
    res = 0.05944
    ox, oy = -66.15462837870969, -58.7392475352799

    def world_to_px(wx, wy):
        px = int((wx - ox) / res)
        py = h_map - 1 - int((wy - oy) / res)
        return px, py

    # Create colorful base track image (900x900 viewport)
    view_size = 920
    track_bgr = np.full((h_map, w_map, 3), (28, 30, 36), dtype=np.uint8)  # Dark asphalt background
    track_mask = (base_gray > 200)
    walls_mask = (base_gray == 0)

    # Free navigable space (light asphalt)
    track_bgr[track_mask] = (220, 222, 225)
    # Walls / boundaries (bright cyan/blue)
    track_bgr[walls_mask] = (70, 70, 75)

    # 5 Static Obstacle locations (from Sepang_obs_map)
    obs_coords = [
        (-24.95, -2.54, "Obs 1"),
        (-22.77, 37.15, "Obs 2"),
        (38.07, 8.48, "Obs 3"),
        (3.01, -39.55, "Obs 4"),
        (21.95, 1.49, "Obs 5")
    ]

    # Pre-draw obstacles on base track with orange/red glow
    for ox_w, oy_w, lbl in obs_coords:
        px, py = world_to_px(ox_w, oy_w)
        cv2.circle(track_bgr, (px, py), 7, (0, 0, 255), -1)      # Red inner
        cv2.circle(track_bgr, (px, py), 12, (0, 140, 255), 2)    # Orange border
        cv2.putText(track_bgr, lbl, (px + 14, py + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

    # Draw Start / Finish line in green
    th0 = -3.05692
    nx, ny = math.sin(th0), -math.cos(th0)
    p1_w = (-2.0 * nx, -2.0 * ny)
    p2_w = (2.0 * nx, 2.0 * ny)
    p1_px, p1_py = world_to_px(p1_w[0], p1_w[1])
    p2_px, p2_py = world_to_px(p2_w[0], p2_w[1])
    cv2.line(track_bgr, (p1_px, p1_py), (p2_px, p2_py), (0, 255, 0), 4)
    cv2.putText(track_bgr, "START/FINISH", (p1_px - 80, p1_py - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Resize track for display window
    scale = view_size / max(h_map, w_map)
    disp_w = int(w_map * scale)
    disp_h = int(h_map * scale)
    disp_base = cv2.resize(track_bgr, (disp_w, disp_h), interpolation=cv2.INTER_AREA)

    # Initialize gym simulation
    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1).unwrapped
    obs, _, done, _ = env.reset(np.array([[0.0, 0.0, th0]]))

    ctrl = FollowTheGapController(speed_scale=1.0)
    scan_fov = 4.7
    scan_beams = 1080
    angle_min = -scan_fov / 2.0
    angle_inc = scan_fov / scan_beams

    finish_p1 = (-4.0 * nx, -4.0 * ny)
    finish_p2 = (4.0 * nx, 4.0 * ny)

    step = 0
    total_lap_dist = 0.0
    prev_x, prev_y = 0.0, 0.0
    completed_laps = 0
    lap_times = []
    lap_start_sim_time = 0.0

    window_name = "F1TENTH Live 10 Laps - Driver: NEBIR NAFIZ MOLLAH"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, view_size + 360, view_size)

    print("=" * 70)
    print("🏎️   F1TENTH LIVE 10-LAP SIMULATION WINDOW OPENED")
    print("👨‍💻  Driver : NEBIR NAFIZ MOLLAH | Group 3")
    print("📍  Circuit: Sepang International Circuit (with 5 Static Obstacles)")
    print("🖥️  Watch the live simulation window on your screen!")
    print("=" * 70 + "\n")

    start_wall = time.time()
    trail_points = []

    while not env.collisions[0] and completed_laps < target_laps:
        # Step simulation
        scans = obs['scans'][0]
        steer, spd = ctrl.process_scan(scans, angle_min, angle_inc)
        obs, _, done, _ = env.step(np.array([[steer, spd]]))
        step += 1

        x = obs['poses_x'][0]
        y = obs['poses_y'][0]
        theta = obs['poses_theta'][0]

        dist_step = math.hypot(x - prev_x, y - prev_y)
        total_lap_dist += dist_step
        sim_t = step * 0.01

        # Check Start/Finish Line Crossing
        if total_lap_dist > 350.0 and intersect((prev_x, prev_y), (x, y), finish_p1, finish_p2):
            completed_laps += 1
            lt = sim_t - lap_start_sim_time
            lap_times.append(lt)
            avg_v = total_lap_dist / lt
            print(f"🏁 Lap {completed_laps:2d}/{target_laps} COMPLETED | Time: {lt:6.2f}s | Avg Speed: {avg_v:4.2f} m/s | Dist: {total_lap_dist:5.1f}m", flush=True)
            lap_start_sim_time = sim_t
            total_lap_dist = 0.0
            trail_points.clear()

        prev_x, prev_y = x, y

        # Render frame at controlled sub-rate for smooth GUI animation
        if step % speed_multiplier == 0:
            frame = disp_base.copy()

            # Map car coordinates to screen pixels
            px_m, py_m = world_to_px(x, y)
            px_d = int(px_m * scale)
            py_d = int(py_m * scale)

            # Record and draw trajectory trail
            trail_points.append((px_d, py_d))
            if len(trail_points) > 120:
                trail_points.pop(0)

            for i_pt in range(1, len(trail_points)):
                alpha = i_pt / len(trail_points)
                c_val = int(255 * alpha)
                cv2.line(frame, trail_points[i_pt - 1], trail_points[i_pt], (c_val, 180, 50), 2)

            # Dynamic Car Vectors
            cos_th = math.cos(theta)
            sin_th = -math.sin(theta)  # Image y is flipped
            fwd = np.array([cos_th, sin_th], dtype=np.float32)
            lat = np.array([-sin_th, cos_th], dtype=np.float32)
            c_pt = np.array([px_d, py_d], dtype=np.float32)

            # Draw subtle racing headlights (replaces whisker lines)
            for b_idx in range(len(scans)//3, 2*len(scans)//3, 20):
                r_val = min(scans[b_idx], 3.5)
                beam_ang = theta + angle_min + b_idx * angle_inc
                lx_w = x + r_val * math.cos(beam_ang)
                ly_w = y + r_val * math.sin(beam_ang)
                lx_p, ly_p = world_to_px(lx_w, ly_w)
                lx_d, ly_d = int(lx_p * scale), int(ly_p * scale)
                cv2.line(frame, (px_d, py_d), (lx_d, ly_d), (120, 220, 255), 1)

            # Authentic F1 Racecar Dimensions (enlarged for crystal-clear visibility)
            car_l = 26.0
            car_w = 15.0

            # 4 Black Rubber Wheels with Silver Hubs
            wheel_l, wheel_w = 8.0, 4.0
            wheel_offsets = [
                (0.32 * car_l, 0.55 * car_w),   # Front Left
                (0.32 * car_l, -0.55 * car_w),  # Front Right
                (-0.32 * car_l, 0.55 * car_w),  # Rear Left
                (-0.32 * car_l, -0.55 * car_w)  # Rear Right
            ]
            for wo_x, wo_y in wheel_offsets:
                w_center = c_pt + fwd * wo_x + lat * wo_y
                w1 = w_center + fwd * (wheel_l / 2) + lat * (wheel_w / 2)
                w2 = w_center + fwd * (wheel_l / 2) - lat * (wheel_w / 2)
                w3 = w_center - fwd * (wheel_l / 2) - lat * (wheel_w / 2)
                w4 = w_center - fwd * (wheel_l / 2) + lat * (wheel_w / 2)
                cv2.fillPoly(frame, [np.int32([w1, w2, w3, w4])], (25, 25, 25))
                cv2.circle(frame, (int(w_center[0]), int(w_center[1])), 1, (200, 200, 200), -1)

            # Aerodynamic Racecar Body (Pointed Nose, Sidepods, Rear)
            nose = c_pt + fwd * (car_l * 0.55)
            f_l = c_pt + fwd * (car_l * 0.22) + lat * (car_w * 0.35)
            pod_l = c_pt - fwd * (car_l * 0.12) + lat * (car_w * 0.48)
            rear_l = c_pt - fwd * (car_l * 0.46) + lat * (car_w * 0.38)
            rear_r = c_pt - fwd * (car_l * 0.46) - lat * (car_w * 0.38)
            pod_r = c_pt - fwd * (car_l * 0.12) - lat * (car_w * 0.48)
            f_r = c_pt + fwd * (car_l * 0.22) - lat * (car_w * 0.35)

            chassis_pts = np.int32([nose, f_l, pod_l, rear_l, rear_r, pod_r, f_r])
            cv2.fillPoly(frame, [chassis_pts], (20, 50, 235))            # Vibrant Racing Red
            cv2.polylines(frame, [chassis_pts], True, (0, 230, 255), 1)  # Yellow Aero Pinstripe

            # Rear Wing / Spoiler
            sp_l = c_pt - fwd * (car_l * 0.52) + lat * (car_w * 0.58)
            sp_r = c_pt - fwd * (car_l * 0.52) - lat * (car_w * 0.58)
            cv2.line(frame, (int(sp_l[0]), int(sp_l[1])), (int(sp_r[0]), int(sp_r[1])), (255, 255, 255), 3)

            # Driver Cockpit & Yellow Helmet
            cockpit = c_pt - fwd * (car_l * 0.05)
            cv2.circle(frame, (int(cockpit[0]), int(cockpit[1])), 3, (15, 15, 15), -1)
            cv2.circle(frame, (int(cockpit[0]), int(cockpit[1])), 2, (0, 220, 255), -1)

            # Twin Headlights
            hl_l = c_pt + fwd * (car_l * 0.46) + lat * (car_w * 0.18)
            hl_r = c_pt + fwd * (car_l * 0.46) - lat * (car_w * 0.18)
            cv2.circle(frame, (int(hl_l[0]), int(hl_l[1])), 1, (255, 255, 255), -1)
            cv2.circle(frame, (int(hl_r[0]), int(hl_r[1])), 1, (255, 255, 255), -1)

            # Build Right-Side Telemetry Scoreboard Panel (360px wide)
            panel = np.full((disp_h, 360, 3), (20, 22, 28), dtype=np.uint8)

            # Header
            cv2.rectangle(panel, (0, 0), (360, 80), (35, 40, 52), -1)
            cv2.putText(panel, "F1TENTH COMPETITION", (20, 32), cv2.FONT_HERSHEY_DUPLEX, 0.72, (0, 220, 255), 2)
            cv2.putText(panel, "NEBIR NAFIZ MOLLAH", (20, 62), cv2.FONT_HERSHEY_DUPLEX, 0.78, (255, 255, 255), 2)

            # Track & Group Info
            cv2.putText(panel, "CIRCUIT : Sepang International", (20, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (180, 185, 195), 1)
            cv2.putText(panel, "GROUP   : Group 3 | 5 Obstacles", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (180, 185, 195), 1)

            # Live Lap Counter (Large Box)
            cv2.rectangle(panel, (20, 160), (340, 245), (32, 45, 65), -1)
            cv2.rectangle(panel, (20, 160), (340, 245), (0, 180, 255), 2)
            cv2.putText(panel, "LAP COUNTER", (35, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 200, 255), 1)
            cv2.putText(panel, f"{completed_laps + 1} / {target_laps}", (35, 232), cv2.FONT_HERSHEY_DUPLEX, 1.25, (255, 255, 255), 2)

            # Live Running Lap Timer
            curr_lap_time = sim_t - lap_start_sim_time
            cv2.rectangle(panel, (20, 260), (340, 345), (28, 50, 42), -1)
            cv2.rectangle(panel, (20, 260), (340, 345), (50, 205, 50), 2)
            cv2.putText(panel, "CURRENT LAP TIMER", (35, 285), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (120, 255, 120), 1)
            cv2.putText(panel, f"{curr_lap_time:6.2f} s", (35, 332), cv2.FONT_HERSHEY_DUPLEX, 1.25, (50, 255, 50), 2)

            # Telemetry Gauges (Speed & Steering & Collisions)
            cv2.putText(panel, f"Speed    : {spd:4.2f} m/s", (25, 385), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 1)
            cv2.putText(panel, f"Steering : {steer:+5.2f} rad", (25, 415), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 1)
            cv2.putText(panel, f"Distance : {total_lap_dist:5.1f} m", (25, 445), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 1)
            cv2.putText(panel, f"Collisions : 0 (Clean)", (25, 475), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2)

            # Infield Lap Scoreboard History
            cv2.rectangle(panel, (20, 505), (340, disp_h - 20), (25, 28, 36), -1)
            cv2.rectangle(panel, (20, 505), (340, disp_h - 20), (70, 75, 90), 1)
            cv2.putText(panel, "LAP-BY-LAP SCOREBOARD", (35, 532), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2)

            y_offset = 565
            for idx_t, t_lap in enumerate(lap_times[-10:], 1):
                best_indicator = " *" if t_lap == min(lap_times) else ""
                cv2.putText(panel, f"Lap {idx_t:2d}:  {t_lap:6.2f} s{best_indicator}", (35, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 225, 235), 1)
                y_offset += 28

            if lap_times:
                cv2.putText(panel, f"FASTEST: {min(lap_times):.2f} s", (35, y_offset + 10), cv2.FONT_HERSHEY_DUPLEX, 0.62, (0, 255, 255), 2)

            # Combine track frame and side panel
            combined = np.hstack([frame, panel])

            cv2.imshow(window_name, combined)
            key = cv2.waitKey(1)
            if key == 27:  # ESC key to exit early
                print("\n[!] Simulation paused by user (ESC key).")
                break

    cv2.destroyAllWindows()
    elapsed_wall = time.time() - start_wall
    print("\n" + "=" * 70)
    if env.collisions[0]:
        print(f"❌ COLLISION OCCURRED at step {step}!")
        return False
    else:
        print(f"🏆 SUCCESS: Completed all {completed_laps} consecutive laps WITHOUT ANY COLLISIONS!")
        print(f"⚡ Fastest Lap  : {min(lap_times):.2f} s")
        print(f"⏱️  Average Lap  : {np.mean(lap_times):.2f} s")
        print(f"🏁 Total Sim Time: {step*0.01:.2f} s (Wall time: {elapsed_wall:.2f} s)")
        print("=" * 70 + "\n")
        return True


if __name__ == '__main__':
    run_live_gui(target_laps=10, speed_multiplier=4)
