#!/usr/bin/env python3

"""
mpc_controller_node.py

Risk-Aware Nonlinear MPC Controller
Trajectory Tracking Version

Author : Majdi Hassan

"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseArray, PoseStamped
from std_msgs.msg import Float32

from tf_transformations import euler_from_quaternion

from .mpc_solver import MPCSolver


class MPCController(Node):


    # Constructor

    def __init__(self):

        super().__init__("mpc_controller_node")


        # Robot State

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.w = 0.0

        # Goal Status


        self.goal_reached = False

        # Distance required before switching to next waypoint
        self.goal_tolerance = 0.20

        # Reference Trajectory Generation
        # Synchronized with Manuscript Figures (Sine Wave)

        self.reference_path = [
            (
                float(x_val),
                float(2.0 + 2.0 * math.sin(0.8 * (0.5 * x_val)))
            )
            for x_val in np.arange(0.0, 7.2, 0.2)
        ]


        # Current Waypoint

        self.reference_index = 0

        self.goal_x = self.reference_path[0][0]
        self.goal_y = self.reference_path[0][1]

        # Collision Risk


        self.collision_risk = 0.0

        # Predicted Obstacles

        self.predicted_obstacles = []


        # MPC Solver Initialization (Table I: Np = 20, Ts = 0.10s)

        self.solver = MPCSolver(
            horizon=20,
            dt=0.10
        )

        # Publishers

        self.cmd_pub = self.create_publisher(
            Twist,
            "/robot1/cmd_vel",
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            "/actual_path",
            10
        )

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"


        # Subscribers

        self.create_subscription(
            Odometry,
            "/robot1/odom",
            self.odom_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/collision_risk",
            self.risk_callback,
            10
        )

        self.create_subscription(
            PoseArray,
            "/predicted_obstacles",
            self.prediction_callback,
            10
        )

        # Controller Timer Loop (Ts = 0.10 s -> 10 Hz)

        self.timer = self.create_timer(
            0.10,
            self.control_loop
        )

        # Startup Information

        self.get_logger().info("")
        self.get_logger().info(" Risk-Aware Nonlinear MPC Controller")
        self.get_logger().info(
            f"Reference Waypoints : {len(self.reference_path)}"
        )
        self.get_logger().info(
            f"Current Goal : ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )
        self.get_logger().info(
            f"Horizon (Np) : {self.solver.N}"
        )
        self.get_logger().info(
            f"Sampling Time (Ts) : {self.solver.dt:.2f} s"
        )
        self.get_logger().info("Controller Ready.")

    # Odometry Callback

    def odom_callback(self, msg: Odometry):

        # Robot Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Robot Orientation
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        (_, _, self.yaw) = euler_from_quaternion(quaternion)

        # Robot Velocity
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

        # Publish Actual Path Visual
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = msg.pose.pose.orientation

        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    # Collision Risk Callback

    def risk_callback(self, msg: Float32):

        self.collision_risk = float(msg.data)

    # Predicted Obstacles Callback

    def prediction_callback(self, msg: PoseArray):

        self.predicted_obstacles.clear()

        for pose in msg.poses:
            self.predicted_obstacles.append(
                (
                    pose.position.x,
                    pose.position.y
                )
            )


    # State Helpers

    def current_state(self):
        return np.array([self.x, self.y, self.yaw], dtype=float)

    def goal_state(self):
        return np.array([self.goal_x, self.goal_y, 0.0], dtype=float)

    def distance_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def heading_error(self):
        desired_heading = math.atan2(
            self.goal_y - self.y,
            self.goal_x - self.x
        )
        error = desired_heading - self.yaw
        while error > math.pi:
            error -= 2.0 * math.pi
        while error < -math.pi:
            error += 2.0 * math.pi
        return error


    # Waypoint Management

    def next_waypoint(self):

        if self.reference_index >= len(self.reference_path) - 1:
            self.goal_reached = True
            self.get_logger().info("Final waypoint reached.")
            return

        self.reference_index += 1
        self.goal_x = self.reference_path[self.reference_index][0]
        self.goal_y = self.reference_path[self.reference_index][1]

        self.get_logger().info(
            f"Waypoint {self.reference_index + 1}/{len(self.reference_path)} "
            f"-> ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def remaining_waypoints(self):
        return len(self.reference_path) - self.reference_index - 1

    def current_waypoint(self):
        return (self.goal_x, self.goal_y)

    def reset_reference_path(self):
        self.reference_index = 0
        self.goal_reached = False
        self.goal_x = self.reference_path[0][0]
        self.goal_y = self.reference_path[0][1]

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


    # Control Loop

    def control_loop(self):

        if self.goal_reached:
            self.stop_robot()
            return

        distance = self.distance_to_goal()

        if distance <= self.goal_tolerance:
            self.next_waypoint()
            if self.goal_reached:
                self.stop_robot()
                self.get_logger().info("Trajectory completed.")
                return
            distance = self.distance_to_goal()

        current_state = self.current_state()
        goal_state = self.goal_state()

        try:
            linear_velocity, angular_velocity = self.solver.solve(
                current_state=current_state,
                goal_state=goal_state,
                collision_risk=self.collision_risk,
                predicted_obstacles=self.predicted_obstacles
            )
        except Exception as error:
            self.get_logger().error(f"MPC Solver Error : {error}")
            self.stop_robot()
            return

        if np.isnan(linear_velocity):
            linear_velocity = 0.0
        if np.isnan(angular_velocity):
            angular_velocity = 0.0

        # Actuator Saturations (Table I Limits: vmax = 1.0 m/s, wmax = 1.5 rad/s)
        linear_velocity = np.clip(
            linear_velocity,
            self.solver.v_min,
            self.solver.v_max
        )

        angular_velocity = np.clip(
            angular_velocity,
            self.solver.w_min,
            self.solver.w_max
        )

        cmd = Twist()
        cmd.linear.x = float(linear_velocity)
        cmd.angular.z = float(angular_velocity)

        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"WP: {self.reference_index + 1}/{len(self.reference_path)} | "
            f"Pos: ({self.x:.2f}, {self.y:.2f}) | "
            f"Risk: {self.collision_risk:.2f} | "
            f"v: {linear_velocity:.3f} | w: {angular_velocity:.3f}"
        )

    ############################################################
    # Utility Methods
    ############################################################

    def set_goal(self, x, y):
        self.reference_path = [(float(x), float(y))]
        self.reference_index = 0
        self.goal_x = float(x)
        self.goal_y = float(y)
        self.goal_reached = False

    def reset_controller(self):
        self.goal_reached = False
        self.reset_reference_path()
        self.collision_risk = 0.0
        self.predicted_obstacles.clear()
        self.solver.reset()

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"
        self.stop_robot()
        self.get_logger().info("Controller Reset.")

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


# Main Entry Point

def main(args=None):

    rclpy.init(args=args)

    controller = MPCController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Controller stopped.")
    except Exception as error:
        controller.get_logger().error(f"Unexpected Error : {error}")
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()