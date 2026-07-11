#!/bin/bash
# ==========================================================
# Micro-ROS Robot Native Teleoperation Launcher
# Uses native Snap micro-ros-agent (no Docker)
# ROS_DOMAIN_ID=0 so agent and ROS 2 nodes share the same domain
# ==========================================================

# Use Domain 0 to match the Snap agent
export ROS_DOMAIN_ID=0
export DISPLAY=:1

# Trap Ctrl+C to clean up background processes
trap "echo 'Shutting down...'; kill -9 $LAUNCH_PID $SLAM_PID $AGENT_PID; exit 0" SIGINT

echo "================================================="
echo "   Micro-ROS Robot Launcher (Domain 0, Native)"
echo "================================================="

# Kill any old agent, free port 8888
killall -9 micro_ros_agent 2>/dev/null; sleep 0.5

echo "[1/3] Starting Native Snap Micro-ROS Agent on Domain 0..."
micro-ros-agent udp4 --port 8888 > /tmp/agent_log.txt 2>&1 &
AGENT_PID=$!
sleep 1

echo ""
echo "🔴🔴🔴 ACTION REQUIRED 🔴🔴🔴"
echo "Please press the EN (Reset) button on your ESP32 board NOW."
echo "Waiting 6 seconds for ESP32 to connect..."
sleep 6

echo ""
echo "[2/3] Launching ROS 2 Robot Nodes (Domain 0)..."
source /opt/ros/humble/setup.bash
source ~/tof_microros_project/ros2_ws/install/setup.bash

ros2 launch esp32_tof_slam robot_complete.launch.py > /tmp/robot_log.txt 2>&1 &
LAUNCH_PID=$!
sleep 4

ros2 launch esp32_tof_slam slam_realtime.launch.py > /tmp/slam_log.txt 2>&1 &
SLAM_PID=$!

echo "[3/3] All nodes launched! RViz should be opening..."
echo "Use teleop_twist_keyboard in a separate terminal to drive."
echo "Press Ctrl+C to stop everything."
echo "================================================="

wait
