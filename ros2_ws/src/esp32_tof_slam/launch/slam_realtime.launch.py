#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    slam_params = {
        'use_sim_time': use_sim_time,
        'base_frame': 'base_footprint',
        'odom_frame': 'odom',
        'map_frame': 'map',
        'scan_topic': '/scan',
        'mode': 'mapping',
        'resolution': 0.05,
        'max_laser_range': 4.0,
        'minimum_travel_distance': 0.10,
        'minimum_travel_heading': 0.10,
        'map_update_interval': 1.0,
        'transform_publish_period': 0.02,
        'tf_buffer_duration': 30.0,
        'use_scan_matching': True,
        'use_scan_barycenter': True,
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),
    ])
