#!/usr/bin/env python3

"""
Digital Twin Node
Digital Twin-Assisted Risk-Aware MPC
Majdi Hassan et al.


Description
-----------
Synchronizes the Digital Twin with the real robot and
extracts obstacle information from the LiDAR.

Functions
---------
1. Receive robot odometry
2. Receive LiDAR measurements
3. Detect obstacles
4. Simulate dynamic obstacles
5. Publish robot state
6. Publish obstacle states
7. Publish RViz markers

Topics
------
Subscribed
----------
/robot1/odom
/robot1/scan

Published
---------
/digital_twin/state
/digital_twin/obstacles
/digital_twin/error
/digital_twin_markers
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import (
    Pose2D,
    Pose,
    PoseArray,
    Point,
)

from visualization_msgs.msg import (
    Marker,
    MarkerArray,
)

from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion


class DigitalTwinNode(Node):


    # Constructor

    def __init__(self):
        super().__init__("digital_twin_node")

        # Sampling Time (Table I: Ts = 0.10 s)
        self.Ts = 0.10
        # LiDAR Parameters

        self.max_lidar_range = 3.50
        self.minimum_range = 0.15


        # Obstacle Clustering

        self.cluster_distance = 0.25
        self.minimum_cluster_size = 3


        # Topics

        self.odom_topic = "/robot1/odom"
        self.scan_topic = "/robot1/scan"

        self.state_topic = "/digital_twin/state"
        self.obstacle_topic = "/digital_twin/obstacles"
        self.error_topic = "/digital_twin/error"
        self.marker_topic = "/digital_twin_markers"

        # Robot State

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Laser Points

        self.lidar_points = []

        # Detected Obstacles

        self.detected_obstacles = []

        # Dynamic Obstacles (Synchronized with Section VI.C)

        self.dynamic_obstacles = [
            # Static Obstacle 1
            {
                "x": 3.5,
                "y": 1.2,
                "vx": 0.00,
                "vy": 0.00
            },
            # Dynamic Obstacle 2
            {
                "x": 2.8,
                "y": 0.8,
                "vx": 0.04,
                "vy": 0.035
            },
            # Dynamic Obstacle 3
            {
                "x": 3.2,
                "y": 2.0,
                "vx": 0.03,
                "vy": 0.025
            }
        ]

        # Subscribers

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )


        # Publishers

        self.state_pub = self.create_publisher(
            Pose2D,
            self.state_topic,
            10
        )

        self.obstacle_pub = self.create_publisher(
            PoseArray,
            self.obstacle_topic,
            10
        )

        self.error_pub = self.create_publisher(
            Float32,
            self.error_topic,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )


        # Main Timer
        self.timer = self.create_timer(
            self.Ts,
            self.update_digital_twin
        )

        # Startup Message

        self.get_logger().info("Digital Twin Node Started.")

    # Odometry Callback


    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        (_, _, self.robot_theta) = euler_from_quaternion(quaternion)


    # LiDAR Callback
    def scan_callback(self, msg):
        self.lidar_points.clear()
        angle = msg.angle_min

        for distance in msg.ranges:
            if (
                math.isinf(distance)
                or math.isnan(distance)
                or distance < self.minimum_range
                or distance > self.max_lidar_range
            ):
                angle += msg.angle_increment
                continue

            x_local = distance * math.cos(angle)
            y_local = distance * math.sin(angle)

            x_world = (
                self.robot_x
                + x_local * math.cos(self.robot_theta)
                - y_local * math.sin(self.robot_theta)
            )

            y_world = (
                self.robot_y
                + x_local * math.sin(self.robot_theta)
                + y_local * math.cos(self.robot_theta)
            )

            point = Point()
            point.x = float(x_world)
            point.y = float(y_world)
            point.z = 0.0

            self.lidar_points.append(point)
            angle += msg.angle_increment

        self.detect_obstacles()

    # LiDAR Clustering

    def detect_obstacles(self):
        self.detected_obstacles.clear()

        if len(self.lidar_points) == 0:
            return

        clusters = []
        current_cluster = [self.lidar_points[0]]

        for i in range(1, len(self.lidar_points)):
            previous = self.lidar_points[i - 1]
            current = self.lidar_points[i]

            distance = math.sqrt(
                (current.x - previous.x) ** 2
                + (current.y - previous.y) ** 2
            )

            if distance < self.cluster_distance:
                current_cluster.append(current)
            else:
                clusters.append(current_cluster)
                current_cluster = [current]

        if len(current_cluster) > 0:
            clusters.append(current_cluster)

        for cluster in clusters:
            if len(cluster) < self.minimum_cluster_size:
                continue

            centroid_x = sum(p.x for p in cluster) / len(cluster)
            centroid_y = sum(p.y for p in cluster) / len(cluster)

            obstacle = Pose()
            obstacle.position.x = float(centroid_x)
            obstacle.position.y = float(centroid_y)
            obstacle.position.z = 0.0
            obstacle.orientation.w = 1.0

            self.detected_obstacles.append(obstacle)

    # Update Dynamic Obstacles

    def update_dynamic_obstacles(self):
        xmin, xmax = 0.5, 8.5
        ymin, ymax = 0.5, 5.5

        for obstacle in self.dynamic_obstacles:
            obstacle["x"] += obstacle["vx"] * self.Ts
            obstacle["y"] += obstacle["vy"] * self.Ts

            # Bounce on boundaries
            if obstacle["x"] < xmin:
                obstacle["x"] = xmin
                obstacle["vx"] *= -1.0
            elif obstacle["x"] > xmax:
                obstacle["x"] = xmax
                obstacle["vx"] *= -1.0

            if obstacle["y"] < ymin:
                obstacle["y"] = ymin
                obstacle["vy"] *= -1.0
            elif obstacle["y"] > ymax:
                obstacle["y"] = ymax
                obstacle["vy"] *= -1.0

    ############################################################
    # Publish Robot State
    ############################################################

    def publish_robot_state(self):
        state = Pose2D()
        state.x = self.robot_x
        state.y = self.robot_y
        state.theta = self.robot_theta
        self.state_pub.publish(state)

    # Publish Obstacles

    def publish_obstacles(self):
        obstacle_array = PoseArray()
        obstacle_array.header.stamp = self.get_clock().now().to_msg()
        obstacle_array.header.frame_id = "odom"

        for obstacle in self.dynamic_obstacles:
            pose = Pose()
            pose.position.x = float(obstacle["x"])
            pose.position.y = float(obstacle["y"])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            obstacle_array.poses.append(pose)

        for obstacle in self.detected_obstacles:
            obstacle_array.poses.append(obstacle)

        self.obstacle_pub.publish(obstacle_array)

    # Publish Synchronization Error

    def publish_error(self):
        error = Float32()
        error.data = 0.0
        self.error_pub.publish(error)

    # Publish RViz Markers

    def publish_markers(self):
        marker_array = MarkerArray()

        # Robot Marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "odom"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.robot_x
        robot_marker.pose.position.y = self.robot_y
        robot_marker.pose.position.z = 0.10
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = 0.25
        robot_marker.scale.y = 0.25
        robot_marker.scale.z = 0.25
        robot_marker.color.r = 0.0
        robot_marker.color.g = 1.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0
        marker_array.markers.append(robot_marker)

        # Dynamic Obstacles Marker
        marker_id = 1
        for obstacle in self.dynamic_obstacles:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dynamic_obstacles"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle["x"]
            marker.pose.position.y = obstacle["y"]
            marker.pose.position.z = 0.10
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.30
            marker.scale.y = 0.30
            marker.scale.z = 0.30
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            marker_id += 1

        # LiDAR Obstacles Marker
        for obstacle in self.detected_obstacles:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "lidar_obstacles"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle.position.x
            marker.pose.position.y = obstacle.position.y
            marker.pose.position.z = 0.10
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.20
            marker.scale.y = 0.20
            marker.scale.z = 0.20
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)

    # Diagnostics

    def print_diagnostics(self):
        self.get_logger().debug(
            f"Robot Position : ({self.robot_x:.2f}, {self.robot_y:.2f})"
        )
        self.get_logger().debug(
            f"Detected LiDAR Obstacles : {len(self.detected_obstacles)}"
        )
        self.get_logger().debug(
            f"Dynamic Obstacles : {len(self.dynamic_obstacles)}"
        )


    # Main Digital Twin Update Loop

    def update_digital_twin(self):
        self.update_dynamic_obstacles()
        self.publish_robot_state()
        self.publish_obstacles()
        self.publish_markers()
        self.publish_error()
        self.print_diagnostics()


# Main Entry Point

def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Digital Twin Node Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()