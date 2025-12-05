# -*- coding: utf-8 -*-
import numpy as np
import L2_vector as vec
import L2_speed_control as sc
import time
from L1_lidar_tim import Lidar  # Make sure this is TIM781 compatible

class HallwayFollower:

    def __init__(self, lidar_ip="192.168.6.241"):
        # LiDAR setup
        self.lidar = Lidar(lidar_ip)
        self.lidar.connect()
        self.lidarControllerThread = self.lidar.run()

        # Robot parameters
        self.base_speed = 0.08
        self.k_p = 0.15
        self.max_speed = 0.25

        self.left_sector = (30, 80)
        self.right_sector = (-80, -30)

        self.min_valid_dist = 0.1
        self.max_valid_dist = 8.0

    def _get_side_distance(self, scan, ang_min, ang_max):
        if scan is None or len(scan) == 0:
            return None
        distances = scan[:,0]
        angles = scan[:,1]
        mask = (angles >= ang_min) & (angles <= ang_max)
        if not np.any(mask):
            return None
        valid = distances[mask]
        valid = valid[(valid > self.min_valid_dist) & (valid < self.max_valid_dist)]
        if valid.size == 0:
            return None
        return float(np.mean(valid))

    def _compute_wheel_speeds(self, d_left, d_right):
        if d_left is None or d_right is None:
            return np.array([self.base_speed, self.base_speed])
        error = d_right - d_left
        correction = self.k_p * error
        left_speed = np.clip(self.base_speed + correction, -self.max_speed, self.max_speed)
        right_speed = np.clip(self.base_speed - correction, -self.max_speed, self.max_speed)
        return np.array([left_speed, right_speed])

    def run(self):
        print("Starting HallwayFollower...")
        try:
            while True:
                scan = self.lidar.get()
                if scan is None:
                    sc.driveOpenLoop(np.array([0.0, 0.0]))
                    time.sleep(0.05)
                    continue

                d_left = self._get_side_distance(scan, *self.left_sector)
                d_right = self._get_side_distance(scan, *self.right_sector)
                wheel_speeds = self._compute_wheel_speeds(d_left, d_right)

                print(f"L: {d_left if d_left else 'None':>5} | "
                      f"R: {d_right if d_right else 'None':>5} | "
                      f"Speeds: [{wheel_speeds[0]:.3f}, {wheel_speeds[1]:.3f}]")

                sc.driveOpenLoop(wheel_speeds)
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("Stopping HallwayFollower.")
            sc.driveOpenLoop(np.array([0.0, 0.0]))
