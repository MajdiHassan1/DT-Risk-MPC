#!/usr/bin/env python3
"""
Launch script for the 5-WMR industrial dep
"""

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_robot_sdf(
    name: str,
    scan_topic: str,
    cmd_topic: str,
    odom_topic: str,
    tf_topic: str,
    odom_frame: str,
    base_frame: str
) -> str:
    """Generates SDF 1.6 description for a differential-drive WMR with 2D LiDAR."""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <!-- Base Chassis Link -->
    <link name="base_link">
      <pose>0 0 0.05 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.05</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.05</iyy><iyz>0.0</iyz><izz>0.08</izz>
        </inertia>
      </inertial>
      <visual name="chassis_visual">
        <geometry><cylinder><radius>0.20</radius><length>0.10</length></cylinder></geometry>
        <material><ambient>0.1 0.3 0.8 1.0</ambient><diffuse>0.1 0.3 0.8 1.0</diffuse></material>
      </visual>
      <collision name="chassis_collision">
        <geometry><cylinder><radius>0.20</radius><length>0.10</length></cylinder></geometry>
      </collision>
    </link>

    <!-- Passive Front Caster Link -->
    <link name="caster">
      <pose>0.15 0 -0.03 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.00001</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.00001</iyy><iyz>0.0</iyz><izz>0.00001</izz>
        </inertia>
      </inertial>
      <visual name="caster_visual">
        <geometry><sphere><radius>0.02</radius></sphere></geometry>
        <material><ambient>0.2 0.2 0.2 1.0</ambient><diffuse>0.2 0.2 0.2 1.0</diffuse></material>
      </visual>
      <collision name="caster_collision">
        <geometry><sphere><radius>0.02</radius></sphere></geometry>
      </collision>
    </link>
    <joint name="caster_joint" type="ball">
      <parent>base_link</parent>
      <child>caster</child>
    </joint>

    <!-- Left Drive Wheel -->
    <link name="left_wheel">
      <pose>-0.05 0.15 0.0 -1.570796 0 0</pose>
      <inertial>
        <mass>0.30</mass>
        <inertia>
          <ixx>0.0005</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.0005</iyy><iyz>0.0</iyz><izz>0.0008</izz>
        </inertia>
      </inertial>
      <visual name="left_wheel_visual">
        <geometry><cylinder><radius>0.05</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1.0</ambient><diffuse>0.1 0.1 0.1 1.0</diffuse></material>
      </visual>
      <collision name="left_wheel_collision">
        <geometry><cylinder><radius>0.05</radius><length>0.03</length></cylinder></geometry>
      </collision>
    </link>
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>

    <!-- Right Drive Wheel -->
    <link name="right_wheel">
      <pose>-0.05 -0.15 0.0 -1.570796 0 0</pose>
      <inertial>
        <mass>0.30</mass>
        <inertia>
          <ixx>0.0005</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.0005</iyy><iyz>0.0</iyz><izz>0.0008</izz>
        </inertia>
      </inertial>
      <visual name="right_wheel_visual">
        <geometry><cylinder><radius>0.05</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1.0</ambient><diffuse>0.1 0.1 0.1 1.0</diffuse></material>
      </visual>
      <collision name="right_wheel_collision">
        <geometry><cylinder><radius>0.05</radius><length>0.03</length></cylinder></geometry>
      </collision>
    </link>
    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>

    <!-- Planar 2D LiDAR (180 deg FOV, 10 Hz, 0.15m - 10.0m Range) -->
    <link name="lidar_link">
      <pose>0 0 0.12 0 0 0</pose>
      <inertial>
        <mass>0.10</mass>
        <inertia>
          <ixx>0.0001</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.0001</iyy><iyz>0.0</iyz><izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual name="lidar_visual">
        <geometry><cylinder><radius>0.04</radius><length>0.04</length></cylinder></geometry>
        <material><ambient>0.0 0.0 0.0 1.0</ambient><diffuse>0.0 0.0 0.0 1.0</diffuse></material>
      </visual>
      <sensor name="lidar" type="ray">
        <update_rate>10</update_rate>
        <topic>{scan_topic}</topic>
        <ray>
          <scan>
            <horizontal>
              <samples>180</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.15</min>
            <max>10.0</max>
          </range>
        </ray>
        <always_on>1</always_on>
        <visualize>false</visualize>
      </sensor>
    </link>
    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
    </joint>

    <!-- Plugins -->
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.30</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <topic>{cmd_topic}</topic>
      <odom_topic>{odom_topic}</odom_topic>
      <tf_topic>{tf_topic}</tf_topic>
      <frame_id>{odom_frame}</frame_id>
      <child_frame_id>{base_frame}</child_frame_id>
    </plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
  </model>
</sdf>"""


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("dt_warehouse_framework")
    depot_world_path = os.path.join(pkg_share, "worlds", "depot_world.sdf")

    gz_resource_path = ":".join([
        os.path.join(pkg_share, "worlds"),
        os.path.join(pkg_share, "models"),
        os.path.expanduser("~/.gz/models"),
        os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ])

    # 1. Start Gazebo
    simulation = ExecuteProcess(
        cmd=["gz", "sim", "-r", depot_world_path],
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": gz_resource_path}
    )

    # 2. ROS-Gazebo Bridge – with ground truth pose for each robot
    bridge_args: List[str] = [
        "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    ]
    for i in range(1, 6):
        r = f"robot{i}"
        bridge_args.extend([
            f"/{r}/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist",
            f"/{r}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/{r}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"/{r}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # Ground truth pose (essential for accurate divergence)
            f"/world/world_d/model/{r}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
        ])

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
        output="screen"
    )

    # 3. Static transforms
    tf_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "0", "0", "0", "map", "odom"],
            output="screen",
        )
    ]

    # 4. Robot spawning – with generous delays to ensure Gazebo is ready
    robot_spawns = [
        {"name": "robot1", "x": "-0.40", "y": "0.00", "delay": 12.0},
        {"name": "robot2", "x": "0.40", "y": "3.00", "delay": 16.0},
        {"name": "robot3", "x": "-1.50", "y": "1.50", "delay": 20.0},
        {"name": "robot4", "x": "1.50", "y": "1.50", "delay": 24.0},
        {"name": "robot5", "x": "0.00", "y": "-2.50", "delay": 28.0},
    ]

    # SDF for robot_state_publisher (optional, but kept for compatibility)
    robot_urdf = """<?xml version="1.0"?>
    <robot name="amr_robot">
      <link name="base_link">
        <visual>
          <geometry><cylinder radius="0.20" length="0.10"/></geometry>
          <material name="blue"><color rgba="0.1 0.3 0.8 1.0"/></material>
        </visual>
      </link>
    </robot>"""

    rsp_nodes = []
    spawn_actions = []

    for r in robot_spawns:
        name = r["name"]

        # Static transform from map to robot's odom frame
        tf_nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[r["x"], r["y"], "0.0", "0", "0", "0", "map", f"{name}/odom"],
            output="screen",
        ))

        # Robot state publisher (optional)
        rsp_nodes.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=f"rsp_{name}",
            namespace=name,
            parameters=[{"robot_description": robot_urdf, "frame_prefix": f"{name}/"}],
            output="screen",
        ))

        # Spawn the robot in Gazebo
        sdf_content = generate_robot_sdf(
            name=name,
            scan_topic=f"/{name}/scan",
            cmd_topic=f"/{name}/cmd_vel",
            odom_topic=f"/{name}/odom",
            tf_topic=f"/{name}/tf",
            odom_frame=f"{name}/odom",
            base_frame=f"{name}/base_link"
        )

        spawn_actions.append(TimerAction(
            period=r["delay"],
            actions=[ExecuteProcess(
                cmd=[
                    "ros2", "run", "ros_gz_sim", "create",
                    "-world", "world_d",   # Correct world name
                    "-name", name,
                    "-string", sdf_content,
                    "-x", r["x"],
                    "-y", r["y"],
                    "-z", "0.05"
                ],
                output="screen",
            )],
        ))

    return LaunchDescription([
        simulation,
        bridge,
        *tf_nodes,
        *rsp_nodes,
        *spawn_actions
    ])