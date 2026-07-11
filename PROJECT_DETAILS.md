# Project Details & Development Journal

This document serves as a deep dive into the architecture, setup, and troubleshooting journey of the ESP32 ToF Micro-ROS SLAM Robot. 

---

## 🏗️ Hardware Architecture & Wiring
The physical robot is built around an ESP32 microcontroller, interfacing with several peripherals.

### Pinout Configuration
* **VL53L0X ToF Sensor:** I2C Bus -> `SDA = GPIO 32`, `SCL = GPIO 33`
* **MPU6050 IMU:** I2C Bus -> `SDA = GPIO 32`, `SCL = GPIO 33`
* **TB6612FNG Motor Driver:**
  * **Motor A (Left):** `IN1 = 18`, `IN2 = 5`, `PWM ENA = 19`
  * **Motor B (Right):** `IN3 = 21`, `IN4 = 22`, `PWM ENB = 23`
  * **Standby:** `STBY = 4` (Must be pulled HIGH for motors to run)
  * **Power:** `VCC = 3.3V` (Logic), `VMOT = 12V` (Battery power for physical movement)
* **Wheel Encoders:**
  * **Left:** `Phase A = 14`, `Phase B = 13`
  * **Right:** `Phase A = 27`, `Phase B = 26`

---

## 🧠 Software Architecture

### ESP32 Firmware (`main.c`)
The firmware is written in C using ESP-IDF v5.1 and heavily leverages FreeRTOS for multitasking. 
To ensure the high-frequency sensor loops don't block the Micro-ROS communications, we utilize the ESP32's dual-core architecture:
* **Core 0:** Runs the physical hardware interrupts (Encoders), I2C polling (IMU/ToF), and the main `app_main()` boot sequence.
* **Core 1:** Pinned specifically for the `micro_ros_task`, handling all UDP Wi-Fi traffic, ROS 2 Publishers, and the `cmd_vel` Subscriber.

### ROS 2 Node Graph
The host computer runs ROS 2 Humble. The data pipeline flows as follows:
1. **`esp32_robot` (Micro-ROS Agent):** Bridges the ESP32 to the ROS 2 network. Publishes `/left_ticks`, `/right_ticks`, `/tof_distance`, and `/imu/data`.
2. **`wheel_odom_node.py`:** Subscribes to the raw encoder ticks, calculates the kinematic math, and publishes the `/wheel/odometry` topic.
3. **`ekf_filter_node` (robot_localization):** Fuses the `/wheel/odometry` and `/imu/data` to produce a highly accurate `odom` -> `base_footprint` TF transform.
4. **`tof_to_laserscan_realtime.py`:** Takes the single integer from `/tof_distance` (in mm) and converts it into a valid `sensor_msgs/LaserScan` message on `/scan`.
5. **`slam_toolbox`:** Subscribes to `/scan` and `/tf`, generating the 2D map (`/map`).
6. **`teleop_twist_keyboard`:** Captures user keystrokes and publishes `geometry_msgs/Twist` directly to `/cmd_vel` for the ESP32 to execute.

---

## 🚧 Problems Faced & Solutions

Building a hybrid Micro-ROS system from scratch presented several complex integration challenges. Here is a step-by-step breakdown of how they were diagnosed and resolved:

### 1. Network & IP Subnet Isolation
**Problem:** The ESP32 was connected to the laptop's Mobile Hotspot (`10.42.0.49`), but could not establish a Micro-ROS handshake with the Docker-based agent.
**Diagnosis:** Docker containers default to their own isolated network bridge, meaning UDP traffic from the ESP32's subnet was being dropped before reaching the agent.
**Solution:** We transitioned from the Docker agent to the native Snap `micro-ros-agent` running directly on the host machine (`10.42.0.1`), ensuring flawless UDP routing.

### 2. The Silent Domain Mismatch
**Problem:** The ESP32 successfully connected to the agent, but running `ros2 topic list` showed absolutely nothing. 
**Diagnosis:** The host machine had an environment variable `ROS_DOMAIN_ID=42` set in its `.bashrc`. However, the Snap micro-ros-agent defaults to `ROS_DOMAIN_ID=0`. The data was flowing perfectly, but the host and agent were on completely different ROS "channels".
**Solution:** We updated the `run_robot.sh` launch script to aggressively force `export ROS_DOMAIN_ID=0` across all terminals, ensuring the entire stack communicates on the same domain.

### 3. Obstacle Avoidance Watchdog Blocking Teleop
**Problem:** Teleop commands were sent, but the robot would only twitch or not move at all in simulation/real life.
**Diagnosis:** We inspected the topic graph (`ros2 topic info /cmd_vel`) and discovered *two* publishers. An experimental `obstacle_avoidance.py` node was running in the background. It had a safety "watchdog" timer that published zero-velocity commands (stop) 20 times a second if it didn't receive constant input on `/cmd_vel_raw`. This was actively overriding the user's manual teleop commands.
**Solution:** Removed the `obstacle_avoidance` node from the `robot_complete.launch.py` file to allow direct, uninhibited teleop control over `/cmd_vel`.

### 4. The "Dancing" Robot (TF Conflict)
**Problem:** In RViz, the robot model was aggressively flickering and jumping back and forth between two locations.
**Diagnosis:** Running `tf2_monitor` revealed multiple identical nodes (`slam_toolbox`, `ekf_node`, `robot_state_publisher`) publishing the same transforms. This occurred because previous launch attempts were closed abruptly (e.g., closing a terminal window), leaving "zombie" ROS 2 processes running invisibly in the background. The new nodes and the zombie nodes were fighting over the robot's location.
**Solution:** Implemented a "nuclear" cleanup command (`killall -9`) to wipe all ROS processes and restart the ROS daemon before every fresh launch.

### 5. The Hardware Power Illusion
**Problem:** The software stack was verified to be 100% operational. The ESP32 logged that it received `cmd_vel` from the laptop and executed the `motors_set()` function. The wheels spun perfectly if turned by hand (encoders worked), but the ROS teleop commands resulted in no physical movement.
**Diagnosis:** The ESP32 was plugged in via USB for debugging. USB provides 5V logic power, which is enough to power the ESP32, the WiFi chip, the encoders, and the logic side of the motor driver. However, the TB6612FNG requires a separate 12V supply on the `VMOT` pin to physically drive the DC motors under load. 
**Solution:** Connected and powered on the 12V LiPo battery. The logic signals finally had the physical current needed to spin the wheels, resulting in successful teleop and SLAM mapping.

---

## 🏆 Conclusion
What made this project successful was strict, methodical isolation of variables. By utilizing `ros2 topic hz`, `tf2_echo`, and custom serial logging via USB, we were able to distinctly separate ROS 2 graph errors, Wi-Fi networking errors, and physical hardware wiring errors, resolving each layer systematically until the entire autonomous stack functioned as a cohesive unit.
