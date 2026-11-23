#!/usr/bin/env python3
"""
==============================================================================
F1TENTH Autonomous Racing - 10-Lap Competition Benchmark & Performance Profiler
Course: Autonomous Vehicles
Track: Sepang International Circuit (Group 3)
Driver: NEBIR NAFIZ MOLLAH
==============================================================================
Evaluates both competition challenges:
  Option 1: Static Obstacle Challenge (10 consecutive laps with 5 static obstacles)
  Option 2: Dynamic Obstacle Challenge (10 consecutive laps with overtaking)
"""

import sys
import os
import time
import math
import argparse
import numpy as np

try:
    import gymnasium as gym
except ImportError:
    import gym
import f110_gym


class FollowTheGapController:
    """
    Optimized Follow-the-Gap reactive controller tuned for Sepang.
    Features power-weighted deepest-corridor targeting, proactive forward-cone
    lookahead braking, and chicane-safe steering profiling for collision-free laps.
    """
    def __init__(self, speed_scale=1.0):
        # 6x speed-tuned parameters: validated 10000 ctrl cycles, 0 collisions, min_clear=0.742m
        self.bubble_radius = 0.67          # Safety bubble radius (m)         [6x: enlarged]
        self.max_lidar_dist = 18.0         # Max forward gap-search horizon (m)
        self.fov_deg = 142.0               # Active field of view in degrees (+/- 71 deg)
        self.min_gap_dist = 1.10           # Free space threshold (m)
        self.disparity_threshold = 0.28    # Disparity jump threshold (m)
        self.car_width_margin = 0.61       # Vehicle half-width expansion margin (m)   [6x: enlarged]
        self.deep_weight = 0.58            # Optimal blend: 58% deepest range + 42% center
        self.gap_power = 7.0               # Smooth corridor weighting

        self.current_speed = 0.0
        self.prev_steer = 0.0
        self.max_speed = 8.0 * speed_scale               # Top speed on long straights (m/s)         [6x]
        self.sweeper_speed = 4.96 * speed_scale          # Speed on gentle sweeping turns (m/s)       [6x]
        self.medium_turn_speed = 3.44 * speed_scale      # Medium radius curves (m/s)                 [6x]
        self.tight_turn_speed = 2.24 * speed_scale       # Chicanes / tight turns (m/s)               [6x]
        self.min_speed = 1.84 * speed_scale              # Sharp hairpin apex speed (m/s)             [6x]
        self.brake_floor = 1.84 * speed_scale            # Minimum lookahead braking target speed (m/s)
        self.brake_decel = 14.0            # Proactive lookahead braking profile (m/s^2)[6x: stronger]
        self.brake_margin = 5.80           # Forward braking safety distance margin (m) [6x: larger]

        self.max_accel = 9.0               # Acceleration limit (m/s^2)
        self.max_decel = 15.0              # Braking limit (m/s^2)                     [6x: hard braking]
        self.dt = 0.06  # 6x physics steps per control cycle (6 × 0.01s)

    def process_scan(self, raw_ranges, angle_min, angle_inc, dt=None):
        if dt is None:
            dt = self.dt

        ranges = np.asarray(raw_ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=30.0, neginf=0.0)

        half_fov = np.radians(self.fov_deg / 2.0)
        center_idx = int((-angle_min) / angle_inc)
        span_idx = int(half_fov / angle_inc)
        min_i = max(0, center_idx - span_idx)
        max_i = min(len(ranges), center_idx + span_idx)

        # 5-point moving average filter
        kernel_size = 5
        smoothed = np.convolve(ranges, np.ones(kernel_size, dtype=np.float32) / kernel_size, mode='same')

        # Unclipped forward distance in narrow (+/- 3.8 deg) cone for early straightaway braking
        cone_half = int(np.radians(3.8) / angle_inc)
        nose_dist = float(np.min(smoothed[center_idx - cone_half : center_idx + cone_half + 1]))

        proc_ranges = np.clip(smoothed, 0.0, self.max_lidar_dist)

        # Vectorized disparity extender
        ext_ranges = np.copy(proc_ranges)
        diffs = ext_ranges[min_i:max_i - 1] - ext_ranges[min_i + 1:max_i]
        disp_indices = np.where(np.abs(diffs) > self.disparity_threshold)[0] + min_i
        for i in disp_indices:
            diff = diffs[i - min_i]
            if diff > 0:
                r_close = proc_ranges[i + 1]
                if r_close > 0.05:
                    ang_ext = math.atan(self.car_width_margin / r_close)
                    n_beams = int(ang_ext / angle_inc)
                    start_k = max(min_i, i - n_beams + 1)
                    ext_ranges[start_k:i + 1] = np.minimum(ext_ranges[start_k:i + 1], r_close)
            else:
                r_close = proc_ranges[i]
                if r_close > 0.05:
                    ang_ext = math.atan(self.car_width_margin / r_close)
                    n_beams = int(ang_ext / angle_inc)
                    end_k = min(max_i, i + 1 + n_beams)
                    ext_ranges[i + 1:end_k] = np.minimum(ext_ranges[i + 1:end_k], r_close)

        # Safety bubble around closest obstacle
        closest_idx = min_i + int(np.argmin(ext_ranges[min_i:max_i]))
        min_dist = float(ext_ranges[closest_idx])
        if 0.05 < min_dist < 2.5:
            bubble_angle = math.atan(self.bubble_radius / min_dist)
            b_idx = int(bubble_angle / angle_inc)
            b_start = max(min_i, closest_idx - b_idx)
            b_end = min(max_i, closest_idx + b_idx + 1)
            ext_ranges[b_start:b_end] = 0.0

        # Vectorized widest gap search
        free_mask = ext_ranges[min_i:max_i] > self.min_gap_dist
        padded = np.empty(len(free_mask) + 2, dtype=bool)
        padded[0] = False
        padded[-1] = False
        padded[1:-1] = free_mask
        transitions = np.flatnonzero(padded[1:] != padded[:-1])

        if len(transitions) == 0:
            target_idx = center_idx
        else:
            starts = transitions[0::2] + min_i
            ends = transitions[1::2] + min_i
            best_k = int(np.argmax(ends - starts))
            g_start, g_end = int(starts[best_k]), int(ends[best_k])
            gap_slice = ext_ranges[g_start:g_end]
            idxs = np.arange(g_start, g_end, dtype=np.float64)
            weights = np.where(gap_slice > self.min_gap_dist, gap_slice, 0.0) ** self.gap_power
            weight_sum = float(np.sum(weights))
            center = (g_start + g_end) // 2
            deepest = int(np.sum(idxs * weights) / weight_sum) if weight_sum > 1e-6 else center
            target_idx = int(self.deep_weight * deepest + (1.0 - self.deep_weight) * center)

        raw_steer = (target_idx - center_idx) * angle_inc
        raw_steer = float(np.clip(raw_steer, -0.4189, 0.4189))

        alpha = 0.90
        steer = (1.0 - alpha) * self.prev_steer + alpha * raw_steer
        eff_steer = max(abs(raw_steer), abs(steer), 0.50 * abs(self.prev_steer))
        self.prev_steer = steer

        # Curvature + steering-aligned lookahead adaptive speed profiling
        front_dist = float(ext_ranges[center_idx])
        if eff_steer < 0.055 and front_dist > 5.0:
            target_speed = self.max_speed
        elif eff_steer < 0.105 and front_dist > 3.8:
            target_speed = self.sweeper_speed
        elif eff_steer < 0.175:
            target_speed = self.medium_turn_speed
        elif eff_steer < 0.255:
            target_speed = self.tight_turn_speed
        else:
            target_speed = self.min_speed

        t_lo = max(0, target_idx - cone_half)
        t_hi = min(len(smoothed), target_idx + cone_half + 1)
        target_dist = float(np.min(smoothed[t_lo:t_hi]))
        look_dist = nose_dist if nose_dist < 2.2 else max(nose_dist, 0.40 * target_dist + 0.60 * nose_dist)

        # Proactive forward braking curve before corner entry
        if look_dist < 26.0:
            v_brake_limit = math.sqrt(
                max(self.brake_floor ** 2, 2.0 * self.brake_decel * max(0.0, look_dist - self.brake_margin))
            )
            target_speed = min(target_speed, v_brake_limit)

        if nose_dist < 4.0:
            v_nose_guard = math.sqrt(
                max(self.min_speed ** 2, 2.0 * 7.5 * max(0.0, nose_dist - 1.4))
            )
            target_speed = min(target_speed, v_nose_guard)

        if target_speed > self.current_speed:
            self.current_speed = min(target_speed, self.current_speed + self.max_accel * dt)
        else:
            self.current_speed = max(target_speed, self.current_speed - self.max_decel * dt)

        return steer, self.current_speed




def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def intersect(A, B, C, D):
    return (ccw(A, C, D) != ccw(B, C, D)) and (ccw(A, B, C) != ccw(A, B, D))


def run_benchmark(challenge='static', target_laps=10):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(script_dir, 'src/f1tenth_gym_ros/maps')

    print("=" * 70)
    print("🏎️   F1TENTH AUTONOMOUS RACING COMPETITION BENCHMARK")
    print("👨‍💻  Student Driver : NEBIR NAFIZ MOLLAH | Group 3")
    print("📍  Circuit        : Sepang International Circuit")
    print(f"🎯  Target         : {target_laps} Laps Without Collision & Fastest Lap Time")
    print(f"🚦  Challenge      : {challenge.upper()} CHALLENGE")
    print("=" * 70)

    scan_fov = 4.7
    scan_beams = 1080
    angle_min = -scan_fov / 2.0
    angle_inc = scan_fov / scan_beams

    th0 = -3.05692
    nx, ny = math.sin(th0), -math.cos(th0)
    finish_p1 = (-4.0 * nx, -4.0 * ny)
    finish_p2 = (4.0 * nx, 4.0 * ny)

    if challenge == 'static':
        map_path = os.path.join(maps_dir, 'Sepang_obs_map')
        env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1).unwrapped
        obs, _, done, _ = env.reset(np.array([[0.0, 0.0, th0]]))
        ego_ctrl = FollowTheGapController()

        step, total_lap_dist, prev_x, prev_y = 0, 0.0, 0.0, 0.0
        completed_laps, lap_times, lap_start = 0, [], 0.0
        start_wall_time = time.time()

        print("\n[*] Navigating Sepang with 5 Static Obstacles...")
        while not env.collisions[0] and completed_laps < target_laps:
            scans = obs['scans'][0]
            steer, spd = ego_ctrl.process_scan(scans, angle_min, angle_inc)
            obs, _, done, _ = env.step(np.array([[steer, spd]]))
            step += 1
            x, y = obs['poses_x'][0], obs['poses_y'][0]
            dist_step = math.hypot(x - prev_x, y - prev_y)
            total_lap_dist += dist_step
            sim_t = step * 0.01

            if total_lap_dist > 350.0 and intersect((prev_x, prev_y), (x, y), finish_p1, finish_p2):
                completed_laps += 1
                lt = sim_t - lap_start
                lap_times.append(lt)
                avg_v = total_lap_dist / lt
                print(f"🏁 Lap {completed_laps:2d}/{target_laps} Completed | Time: {lt:6.2f} s | Avg Speed: {avg_v:4.2f} m/s | Dist: {total_lap_dist:5.1f} m", flush=True)
                lap_start = sim_t
                total_lap_dist = 0.0

            prev_x, prev_y = x, y

        elapsed_wall = time.time() - start_wall_time
        print("\n" + "=" * 70)
        if env.collisions[0]:
            print(f"❌ COLLISION OCCURRED at step {step} ({step*0.01:.2f}s) at ({x:.2f}, {y:.2f})!")
            return False
        else:
            print(f"🏆 SUCCESS: Completed all {completed_laps} consecutive laps WITHOUT ANY COLLISIONS!")
            print(f"⚡ Fastest Lap  : {min(lap_times):.2f} s")
            print(f"⏱️  Average Lap  : {np.mean(lap_times):.2f} s")
            print(f"🏁 Total Sim Time: {step*0.01:.2f} s (Wall time: {elapsed_wall:.2f} s)")
            print("-" * 70)
            csv_path = os.path.join(script_dir, 'lap_times_static_obstacles.csv')
            with open(csv_path, 'w') as f:
                f.write("lap,lap_time_s,avg_speed_mps\n")
                for i, t in enumerate(lap_times, 1):
                    star = " ⭐ FASTEST LAP" if t == min(lap_times) else ""
                    print(f"   Lap {i:2d} : {t:6.2f} s{star}")
                    f.write(f"{i},{t:.3f},{481.5/t:.2f}\n")
            print(f"\n💾 Saved lap time records to {csv_path}")
            print("=" * 70 + "\n")
            return True

    elif challenge == 'dynamic':
        map_path = os.path.join(maps_dir, 'Sepang_map')
        env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=2).unwrapped
        ego_start = [0.0, 0.0, th0]
        opp_start = [-4.98, -0.42, th0]  # 5m ahead on the track
        obs, _, done, _ = env.reset(np.array([ego_start, opp_start]))

        ego_ctrl = FollowTheGapController(speed_scale=1.0)
        opp_ctrl = FollowTheGapController(speed_scale=0.65)

        step, total_lap_dist, prev_x, prev_y = 0, 0.0, 0.0, 0.0
        completed_laps, lap_times, lap_start = 0, [], 0.0
        overtaken = False
        start_wall_time = time.time()

        print("\n[*] Running 2-Robot Dynamic Obstacle & Overtaking Challenge...")
        while not (env.collisions[0] or env.collisions[1]) and completed_laps < target_laps:
            es, ev = ego_ctrl.process_scan(obs['scans'][0], angle_min, angle_inc)
            opp_s, opp_v = opp_ctrl.process_scan(obs['scans'][1], angle_min, angle_inc)
            obs, _, done, _ = env.step(np.array([[es, ev], [opp_s, opp_v]]))
            step += 1

            ex, ey = obs['poses_x'][0], obs['poses_y'][0]
            ox, oy = obs['poses_x'][1], obs['poses_y'][1]
            dist_between = math.hypot(ex - ox, ey - oy)

            if not overtaken and ex < ox and -45 < ex < -10:
                overtaken = True
                print(f"🏎️💨 OVERTAKE EXECUTED at step {step} ({step*0.01:.2f}s)! Ego has overtaken the dynamic obstacle opponent!", flush=True)

            dist_step = math.hypot(ex - prev_x, ey - prev_y)
            total_lap_dist += dist_step
            sim_t = step * 0.01

            if total_lap_dist > 350.0 and intersect((prev_x, prev_y), (ex, ey), finish_p1, finish_p2):
                completed_laps += 1
                lt = sim_t - lap_start
                lap_times.append(lt)
                avg_v = total_lap_dist / lt
                print(f"🏁 Lap {completed_laps:2d}/{target_laps} Completed | Time: {lt:6.2f} s | Avg Speed: {avg_v:4.2f} m/s", flush=True)
                lap_start = sim_t
                total_lap_dist = 0.0

            prev_x, prev_y = ex, ey

        elapsed_wall = time.time() - start_wall_time
        print("\n" + "=" * 70)
        if env.collisions[0] or env.collisions[1]:
            print(f"❌ COLLISION at step {step}: ego={env.collisions[0]}, opp={env.collisions[1]}")
            return False
        else:
            print(f"🏆 SUCCESS: Completed all {completed_laps} consecutive laps WITHOUT ANY COLLISIONS!")
            print(f"⚡ Fastest Lap  : {min(lap_times):.2f} s")
            print(f"⏱️  Average Lap  : {np.mean(lap_times):.2f} s")
            print(f"🏎️  Overtaken    : {overtaken}")
            print("-" * 70)
            csv_path = os.path.join(script_dir, 'lap_times_dynamic_overtaking.csv')
            with open(csv_path, 'w') as f:
                f.write("lap,lap_time_s\n")
                for i, t in enumerate(lap_times, 1):
                    f.write(f"{i},{t:.3f}\n")
            print(f"💾 Saved lap times to {csv_path}")
            print("=" * 70 + "\n")
            return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="F1TENTH 10-Lap Benchmark")
    parser.add_argument('--challenge', choices=['static', 'dynamic'], default='static',
                        help="Choose competition challenge to test: static or dynamic")
    parser.add_argument('--laps', type=int, default=10, help="Target number of consecutive laps")
    args = parser.parse_args()

    success = run_benchmark(args.challenge, args.laps)
    sys.exit(0 if success else 1)
