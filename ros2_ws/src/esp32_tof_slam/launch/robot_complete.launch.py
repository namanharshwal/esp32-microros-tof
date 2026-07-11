#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = os.path.join(
        os.path.expanduser("~"),
        "tof_microros_project/ros2_ws/src/esp32_tof_slam")

    urdf_path = os.path.join(pkg, "urdf", "tof_robot.urdf.xacro")
    ekf_cfg   = os.path.join(pkg, "config", "ekf.yaml")
    rviz_cfg  = os.path.join(pkg, "rviz", "slam_config.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str)

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }]
        ),

        Node(
            package="esp32_tof_slam",
            executable="wheel_odom_node.py",
            name="wheel_odom_node",
            output="screen"
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_cfg]
        ),

        Node(
            package="esp32_tof_slam",
            executable="tof_to_laserscan_realtime.py",
            name="tof_to_laserscan",
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg] if os.path.exists(rviz_cfg) else [],
            output="screen"
        ),
    ])
