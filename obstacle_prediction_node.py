#!/usr/bin/env python3

"""
obstacle_prediction_node.py

Obstacle Prediction Node
Digital Twin-Assisted Risk-Aware MPC
Synchronized with Manuscript: Paper 2_Majdi Hassan_V2.pdf

Author : Majdi Hassan et al.

Description
-----------
This node predicts the future motion of dynamic obstacles over
the MPC prediction horizon (Np = 20, Ts = 0.10 s) using a
constant-velocity kinematic prediction model.

Functions
---------
1. Receive obstacle positions from Digital Twin (/digital_twin/obstacles)
2. Estimate velocity components (vx, vy) via finite differences
3. Forecast future trajectories over Np steps (Eqs. 15-16)
4. Publish predicted obstacle poses (/predicted_obstacles)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray


class ObstaclePredictionNode(Node):

    # Constructor

    def __init__(self):
        super().__init__("obstacle_prediction_node")

        # Prediction Parameters (Table I Alignment)
        self.Ts = 0.10  # Sampling Period (Ts = 0.10 s)
        self.Np = 20    # Prediction Horizon (Np = 20)

        # Topics
        self.obstacle_topic = "/digital_twin/obstacles"
        self.prediction_topic = "/predicted_obstacles"

        # State Buffers
        self.current_obstacles = []
        self.previous_obstacles = []
        self.obstacle_velocities = []

        # Subscribers
        self.create_subscription(
            PoseArray,
            self.obstacle_topic,
            self.obstacle_callback,
            10
        )

        # Publishers
        self.prediction_pub = self.create_publisher(
            PoseArray,
            self.prediction_topic,
            10
        )

        # Main Timer Loop (10 Hz / Ts = 0.10 s)
        self.timer = self.create_timer(
            self.Ts,
            self.predict_obstacles
        )
#
        # Startup Message
        self.get_logger().info("Obstacle Prediction Node Started.")

    # Obstacle Telemetry Callback

    def obstacle_callback(self, msg):
        self.previous_obstacles = self.current_obstacles
        self.current_obstacles = list(msg.poses)
        self.update_obstacle_velocities()

    # Velocity Estimation (Finite Difference)

    def update_obstacle_velocities(self):
        self.obstacle_velocities.clear()

        # First measurement fallback (Zero Initial Velocity)
        if len(self.previous_obstacles) == 0:
            for _ in self.current_obstacles:
                self.obstacle_velocities.append((0.0, 0.0))
            return

        number_to_process = min(
            len(self.current_obstacles),
            len(self.previous_obstacles)
        )

        # Numerical differentiation across Ts
        for i in range(number_to_process):
            current = self.current_obstacles[i]
            previous = self.previous_obstacles[i]

            vx = (current.position.x - previous.position.x) / self.Ts
            vy = (current.position.y - previous.position.y) / self.Ts

            self.obstacle_velocities.append((vx, vy))

        # Pad newly spawned obstacles with zero velocity
        while len(self.obstacle_velocities) < len(self.current_obstacles):
            self.obstacle_velocities.append((0.0, 0.0))

    # Single Obstacle Constant-Velocity Projection (Eqs. 15-16)

    def predict_single_obstacle(self, obstacle, velocity, prediction_msg):
        x = obstacle.position.x
        y = obstacle.position.y
        vx, vy = velocity

        # Project positions across Np future horizon steps
        for k in range(1, self.Np + 1):
            pose = Pose()
            pose.position.x = float(x + vx * k * self.Ts)
            pose.position.y = float(y + vy * k * self.Ts)
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            prediction_msg.poses.append(pose)

    # Main Prediction & Publishing Loop

    def predict_obstacles(self):
        prediction_msg = PoseArray()
        prediction_msg.header.stamp = self.get_clock().now().to_msg()
        prediction_msg.header.frame_id = "odom"

        # Empty array if no obstacles detected
        if len(self.current_obstacles) == 0:
            self.prediction_pub.publish(prediction_msg)
            return

        # Predict future trajectory for each detected obstacle
        for obstacle, velocity in zip(self.current_obstacles, self.obstacle_velocities):
            self.predict_single_obstacle(obstacle, velocity, prediction_msg)

        # Publish predicted PoseArray
        self.prediction_pub.publish(prediction_msg)

        self.get_logger().debug(
            f"Detected: {len(self.current_obstacles)} | "
            f"Horizon: {self.Np} | "
            f"Total Projected Poses: {len(prediction_msg.poses)}"
        )


# Main Entry Point

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePredictionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Obstacle Prediction Node Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()