#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg = os.path.join(os.path.expanduser('~'),
                       'tof_microros_project/ros2_ws/src/esp32_tof_slam')

    urdf_path = os.path.join(pkg, 'urdf', 'tof_robot.urdf.xacro')
    robot_desc = xacro.process_file(urdf_path).toxml()
    world_path = os.path.join(pkg, 'worlds', 'tof_world.world')
    rviz_cfg   = os.path.join(pkg, 'rviz', 'slam_config.rviz')

    gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 1. Gazebo with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_path}.items()),

        # 2. Robot State Publisher
        Node(package='robot_state_publisher',
             executable='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'robot_description': robot_desc}]),

        # 3. Spawn robot in Gazebo
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description',
                        '-entity', 'tof_robot',
                        '-x', '0', '-y', '0', '-z', '0.1'],
             output='screen'),



        # 5. EKF Sensor Fusion
        Node(package='robot_localization',
             executable='ekf_node', name='ekf_filter_node',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'ekf.yaml'),
                         {'use_sim_time': use_sim_time}]),

        # 6. SLAM Toolbox
        Node(package='slam_toolbox',
             executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[{
                 'use_sim_time':  use_sim_time,
                 'base_frame':   'base_footprint',
                 'odom_frame':   'odom',
                 'map_frame':    'map',
                 'scan_topic':   '/scan',
                 'mode':         'mapping',
                 'resolution':   0.05,
                 'max_laser_range': 4.0,
             }]),

        # 7. RViz
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
             parameters=[{'use_sim_time': use_sim_time}],
             output='screen'),
    ])
