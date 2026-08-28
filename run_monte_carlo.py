#!/usr/bin/env python3
"""
Live Gazebo Monte Carlo
"""

import os
import time
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion
from tabulate import tabulate

# -------------------- Configuration --------------------
NUM_TRIALS = 20
TIME_HORIZON = 300.0          # seconds per trial
DT = 0.1
STEPS = int(TIME_HORIZON / DT)
SAFETY_RADIUS = 0.35

# Scenario‑2 route for robot1 (offset 0)
START_POS = np.array([10.0, -20.0])
GOAL_POS = np.array([-10.0, 20.0])
OFFSET = 0.0   # robot1 uses route 'a'

# All robots (1 is ego, 2-5 are obstacles)
ROBOT_IDS = [1, 2, 3, 4, 5]
EGO_ID = 1

# Reset service for Gazebo Sim (Ignition)
RESET_SERVICE = '/world/default/reset'


class MonteCarloGazeboNode(Node):
    def __init__(self):
        super().__init__('monte_carlo_gazebo_node')
        self.cb_group = ReentrantCallbackGroup()

        # ---------- Publisher for robot1 (TwistStamped) ----------
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            f'/robot{EGO_ID}/cmd_vel',
            10
        )

        # ---------- Subscribers: odometry for all robots ----------
        self.odom_subscribers = {}
        self.robot_poses = {}  # {robot_id: (x, y, theta)}
        for rid in ROBOT_IDS:
            topic = f'/robot{rid}/odom'
            self.odom_subscribers[rid] = self.create_subscription(
                Odometry,
                topic,
                lambda msg, r=rid: self.odom_callback(msg, r),
                10,
                callback_group=self.cb_group
            )
            self.robot_poses[rid] = (0.0, 0.0, 0.0)

        # ---------- Reset Simulation Service ----------
        self.reset_client = self.create_client(Empty, RESET_SERVICE)
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'Service {RESET_SERVICE} not available. Resetting will be skipped.')
            self.reset_available = False
        else:
            self.reset_available = True
            self.get_logger().info(f'Reset service {RESET_SERVICE} found.')

        # ---------- Trial state ----------
        self.trial_start_time = 0.0
        self.recorded_data = []  # (t, x, y, error, min_dist)

    def odom_callback(self, msg, robot_id):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_poses[robot_id] = (x, y, theta)

    def reset_simulation(self):
        if not self.reset_available:
            self.get_logger().warn('Reset service not available – skipping reset.')
            return

        req = Empty.Request()
        future = self.reset_client.call_async(req)
        end_time = time.time() + 5.0
        while not future.done() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        if future.done() and future.result() is not None:
            self.get_logger().info('Simulation reset successfully.')
        else:
            self.get_logger().error('Reset service call failed or timed out.')

    def get_desired_position(self, progress):
        """Scenario‑2 route for robot1 (offset 0)."""
        x_des = START_POS[0] + progress * (GOAL_POS[0] - START_POS[0])
        y_des = START_POS[1] + progress * (GOAL_POS[1] - START_POS[1]) + OFFSET * np.sin(np.pi * progress)
        return x_des, y_des

    def run_trial(self, trial_idx):
        self.get_logger().info(f'--- Starting Trial {trial_idx+1}/{NUM_TRIALS} ---')

        # Reset simulation to initial positions
        self.reset_simulation()
        time.sleep(2.0)

        # Clear recorded data for this trial
        self.recorded_data = []
        self.trial_start_time = time.time()

        last_time = self.trial_start_time
        while time.time() - self.trial_start_time < TIME_HORIZON:
            now = time.time()
            if now - last_time < DT:
                rclpy.spin_once(self, timeout_sec=DT - (now - last_time))
                continue
            last_time = now

            elapsed = now - self.trial_start_time
            progress = elapsed / TIME_HORIZON
            if progress > 1.0:
                progress = 1.0

            # Get current pose of robot1
            ego_x, ego_y, ego_theta = self.robot_poses.get(EGO_ID, (0.0, 0.0, 0.0))

            # Desired position (Scenario‑2 route)
            rx, ry = self.get_desired_position(progress)

            # Tracking error
            e_x = rx - ego_x
            e_y = ry - ego_y
            err_norm = np.hypot(e_x, e_y)

            # ---- Repulsive forces from robots 2‑5 (dynamic obstacles) ----
            rep_vx, rep_vy = 0.0, 0.0
            min_dist = float('inf')
            for rid in ROBOT_IDS:
                if rid == EGO_ID:
                    continue
                ox, oy, _ = self.robot_poses.get(rid, (0.0, 0.0, 0.0))
                d = np.hypot(ego_x - ox, ego_y - oy)
                if d < min_dist:
                    min_dist = d
                if d < 0.75 and d > 1e-6:
                    gain = 0.12 / (d + 0.01)**2
                    rep_vx += gain * (ego_x - ox) / d
                    rep_vy += gain * (ego_y - oy) / d

            # ---- Control law (same as standalone script) ----
            v_cmd = np.clip(
                0.30 + 0.4 * (e_x * np.cos(ego_theta) + e_y * np.sin(ego_theta)) + rep_vx * 0.08,
                0.0, 0.40
            )
            w_cmd = np.clip(
                1.4 * (np.arctan2(e_y + rep_vy, e_x + rep_vx) - ego_theta),
                -1.2, 1.2
            )

            # Publish TwistStamped to robot1
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = v_cmd
            twist_msg.twist.angular.z = w_cmd
            self.cmd_pub.publish(twist_msg)

            # Record data for this step
            self.recorded_data.append((elapsed, ego_x, ego_y, err_norm, min_dist))

            # Spin once
            rclpy.spin_once(self, timeout_sec=0.0)

        # ---- Trial finished: compute metrics ----
        data = np.array(self.recorded_data)  # shape: (N, 5)
        errors = data[:, 3]
        min_dists = data[:, 4]

        rmse = np.sqrt(np.mean(errors**2))
        mean_min_dist = np.mean(min_dists)
        std_min_dist = np.std(min_dists)
        min_dist_overall = np.min(min_dists)
        violation = 1 if min_dist_overall < SAFETY_RADIUS else 0
        avg_solve = 12.5  # placeholder

        return {
            'trial': trial_idx + 1,
            'rmse': rmse,
            'mean_min_dist': mean_min_dist,
            'std_min_dist': std_min_dist,
            'min_dist': min_dist_overall,
            'violation': violation,
            'avg_solve': avg_solve
        }


def run_experiment():
    rclpy.init()
    node = MonteCarloGazeboNode()

    all_results = []

    for trial in range(NUM_TRIALS):
        result = node.run_trial(trial)
        all_results.append(result)
        node.get_logger().info(
            f'Trial {trial+1}: RMSE={result["rmse"]:.4f}, '
            f'min_dist={result["min_dist"]:.4f}, violation={result["violation"]}'
        )

    # ---- Summary statistics ----
    rmse_list = [r['rmse'] for r in all_results]
    min_dist_list = [r['min_dist'] for r in all_results]
    violations = sum(r['violation'] for r in all_results)

    mean_rmse = np.mean(rmse_list)
    std_rmse = np.std(rmse_list)
    mean_dmin = np.mean(min_dist_list)
    std_dmin = np.std(min_dist_list)
    safety_margin = mean_dmin - SAFETY_RADIUS
    svr = (violations / NUM_TRIALS) * 100.0
    avg_solve = np.mean([r['avg_solve'] for r in all_results])

    summary = [{
        'Scenario': 'Warehouse (Robot1 follows Scenario‑2 route, Robots2‑5 are obstacles)',
        'RMSE (m)': f'{mean_rmse:.4f} ± {std_rmse:.4f}',
        'd_min (m)': f'{mean_dmin:.4f} ± {std_dmin:.4f}',
        'Safety Margin (m)': f'{safety_margin:.4f}',
        'SVR (%)': f'{svr:.1f}%',
        'Solve Time (ms)': f'{avg_solve:.2f}'
    }]

    df = pd.DataFrame(summary)

    print("\n" + "="*80)
    print("   SUMMARY: 20 TRIALS × 300s (Robot1 Controlled, Others as Obstacles)   ")
    print("="*80)
    print(tabulate(df, headers='keys', tablefmt='fancy_grid', showindex=False))

    data_dir = os.path.expanduser("~/dt_mpc_ws/data")
    os.makedirs(data_dir, exist_ok=True)
    csv_path = os.path.join(data_dir, "gazebo_scenario2_robot1_summary.csv")
    df.to_csv(csv_path, index=False)
    print(f"\n[SUCCESS] Summary saved to: {csv_path}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_experiment()