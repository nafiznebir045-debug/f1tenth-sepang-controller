#!/usr/bin/env python3
"""
==============================================================================
F1TENTH Dynamic Obstacle Controller (Opponent Vehicle)
Course: Autonomous Vehicles
Track: Sepang International Circuit (Group 3)
Vehicle: opp_racecar
==============================================================================
Runs a stable Follow-the-Gap reactive controller subscribing to /opp_scan
and publishing to /opp_drive, serving as a dynamic obstacle for Challenge 2.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class OpponentReactiveNode(Node):
    def __init__(self):
        super().__init__('opp_reactive_node')

        self.scan_sub = self.create_subscription(
            LaserScan, '/opp_scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/opp_drive', 10)

        # Controller parameters
        self.bubble_radius = 0.40
        self.max_lidar_dist = 10.0
        self.fov_deg = 120.0
        self.min_gap_dist = 0.85
        self.car_width_margin = 0.35
        self.disparity_threshold = 0.25
        self.prev_steer = 0.0
        self.speed = 2.0  # Steady pace allowing ego to overtake

        self.get_logger().info('Opponent Reactive Controller initialized on /opp_scan -> /opp_drive (speed=2.0 m/s)')

    def scan_callback(self, data):
        ranges = np.nan_to_num(np.array(data.ranges, dtype=np.float32), nan=0.0, posinf=self.max_lidar_dist, neginf=0.0)
        half_fov = math.radians(self.fov_deg / 2.0)
        center_idx = int((-data.angle_min) / data.angle_increment)
        span_idx = int(half_fov / data.angle_increment)
        min_i = max(0, center_idx - span_idx)
        max_i = min(len(ranges), center_idx + span_idx)

        # 5-beam moving average smoothing
        kernel = 5
        smoothed = np.convolve(ranges, np.ones(kernel) / kernel, mode='same')
        proc = np.clip(smoothed, 0.0, self.max_lidar_dist)

        # Disparity extender
        ext = np.copy(proc)
        for i in range(min_i, max_i - 1):
            diff = ext[i] - ext[i + 1]
            if abs(diff) > self.disparity_threshold:
                if diff > 0:
                    rc = ext[i + 1]
                    if rc > 0.05:
                        nb = int(math.atan(self.car_width_margin / rc) / data.angle_increment)
                        ext[max(min_i, i - nb + 1):i + 1] = np.minimum(ext[max(min_i, i - nb + 1):i + 1], rc)
                else:
                    rc = ext[i]
                    if rc > 0.05:
                        nb = int(math.atan(self.car_width_margin / rc) / data.angle_increment)
                        ext[i + 1:min(max_i, i + 1 + nb)] = np.minimum(ext[i + 1:min(max_i, i + 1 + nb)], rc)

        # Safety bubble around closest obstacle
        closest_rel = int(np.argmin(ext[min_i:max_i]))
        closest_idx = min_i + closest_rel
        min_d = float(ext[closest_idx])
        if 0.05 < min_d < 2.0:
            b_ang = math.atan(self.bubble_radius / min_d)
            b_idx = int(b_ang / data.angle_increment)
            ext[max(min_i, closest_idx - b_idx):min(max_i, closest_idx + b_idx + 1)] = 0.0

        # Find widest gap
        free_mask = ext[min_i:max_i] > self.min_gap_dist
        gaps, in_gap, start = [], False, 0
        for idx, val in enumerate(free_mask):
            if val and not in_gap:
                in_gap, start = True, idx
            elif not val and in_gap:
                in_gap = False
                gaps.append((start + min_i, idx + min_i))
        if in_gap:
            gaps.append((start + min_i, max_i))

        if not gaps:
            target_idx = center_idx
        else:
            best_gap = max(gaps, key=lambda g: g[1] - g[0])
            deepest = best_gap[0] + int(np.argmax(ext[best_gap[0]:best_gap[1]]))
            center = (best_gap[0] + best_gap[1]) // 2
            target_idx = int(0.50 * deepest + 0.50 * center)

        raw_steer = (target_idx - center_idx) * data.angle_increment
        raw_steer = float(np.clip(raw_steer, -0.4189, 0.4189))
        steer = 0.20 * self.prev_steer + 0.80 * raw_steer
        self.prev_steer = steer

        # Speed adjustment
        front_d = float(proc[center_idx])
        if abs(steer) > 0.20 or front_d < 2.0:
            speed = 1.6
        else:
            speed = self.speed

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'opp_racecar/base_link'
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentReactiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
