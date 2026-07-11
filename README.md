# ESP32 Micro-ROS ToF SLAM Robot

A complete, end-to-end autonomous robotics project built on an ESP32 microcontroller using FreeRTOS, Micro-ROS, and ROS 2 Humble. This robot is capable of real-time 2D mapping (SLAM), accurate odometry via wheel encoders and an IMU, obstacle detection via a Time-of-Flight (ToF) sensor, and remote teleoperation.

## 🚀 Features
* **Micro-ROS Integration:** Direct UDP Wi-Fi communication between the ESP32 and the ROS 2 host.
* **Real-time SLAM:** Uses `slam_toolbox` to generate a 2D map of the environment using a single-point ToF sensor mapped to a LaserScan.
* **Sensor Fusion:** Utilizes `robot_localization` (EKF) to fuse MPU6050 IMU data with wheel encoder odometry for highly accurate positioning.
* **Dual-Core Processing:** Firmware intelligently divides FreeRTOS tasks (Sensors on Core 0, Micro-ROS on Core 1) for maximum performance.
* **Automated Launch System:** Custom bash scripts for one-click deployment of the agent, nodes, and RViz.

---

## 🛠️ Hardware Requirements
* **Microcontroller:** ESP32 (Standard)
* **Motor Driver:** TB6612FNG (Requires 12V Battery on `VMOT`)
* **Sensors:** 
  * VL53L0X Time-of-Flight (ToF) Distance Sensor
  * MPU6050 6-DOF IMU
  * 2x Quadrature Wheel Encoders (DC Motors)
* **Power:** 12V LiPo Battery for Motors, 5V Logic Power.

---

## 💻 Software Stack
* **Firmware:** ESP-IDF v5.1 (C/FreeRTOS)
* **Host OS:** Ubuntu 22.04 LTS (Linux)
* **ROS 2:** Humble Hawksbill
* **ROS 2 Packages:** `slam_toolbox`, `robot_localization`, `teleop_twist_keyboard`, `rviz2`

---

## ⚙️ Quick Start Guide

### 1. Configure the ESP32 (Wi-Fi & Agent IP)
Before building, you must configure the ESP32 to connect to your local Wi-Fi and point it to the Micro-ROS Agent's IP address.

```bash
# Source the ESP-IDF v5.1 environment
. ~/esp/esp-idf-v5.1/export.sh

cd ~/tof_microros_project

# Open the configuration menu
idf.py menuconfig
```
* **Wi-Fi Setup:** Go to `micro-ROS Configuration` -> `WiFi Configuration` and enter your SSID and Password.
* **Agent IP:** Go to `micro-ROS Configuration` -> `micro-ROS Agent IP` and enter the IP address of your host computer (e.g., `10.42.0.1`). Save (S) and Quit (Q).

### 2. Build and Flash the ESP32 DevKit
Ensure the ESP32 is plugged in via USB and the **12V battery is disconnected** during flashing to prevent power surges.

```bash
# Build, flash, and open the serial monitor
idf.py build flash monitor
```
*🛑 **DevKit Note:** If you see `Connecting...` stalling during upload, press and hold the **BOOT** button on your ESP32 DevKit until the upload begins.*

### 2. Clean Environment (Recommended)
Before launching, ensure there are no "zombie" ROS 2 processes running from previous sessions that could cause TF (Transform) conflicts in RViz.
```bash
killall -9 robot_state_publisher ekf_node async_slam_toolbox_node teleop_twist_keyboard rviz2 micro-ros-agent
pkill -9 -f "wheel_odom_node|tof_to_laserscan"
ros2 daemon stop && ros2 daemon start
```

### 3. Launch the Robot Stack
Turn on your robot's 12V battery and run the automated launcher.
```bash
cd ~/tof_microros_project
./run_robot.sh
```
*🛑 **Important:** When prompted by the script (`ACTION REQUIRED`), press the `EN` (Reset) button on the ESP32 to connect it to the Micro-ROS Agent.*

### 4. Drive and Map!
Once RViz opens and the robot model is visible, open a **new** terminal to control the robot:
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use `i`, `,`, `j`, `l`, and `k` to drive the robot around. As you move, SLAM will automatically build the map in RViz!

---

*For an in-depth breakdown of the project architecture, hardware pins, node graphs, and troubleshooting steps, please read [PROJECT_DETAILS.md](PROJECT_DETAILS.md).*
