# Comprehensive Guide: ESP32 Micro-ROS ToF SLAM Robot (The Definitive Manual)

This document serves as the ultimate, definitive guide to the ESP32 Micro-ROS ToF SLAM Robot. It expands upon the basic setup and delves deeply into the mathematical, architectural, and systemic details of the project. This manual is designed for advanced robotics developers, researchers, and hobbyists who want to understand the exact mechanics of how a microcontroller can execute full ROS 2 Simultaneous Localization and Mapping (SLAM) over a Wi-Fi network without a companion single-board computer (like a Raspberry Pi).

With over exhaustive detail on every hardware component, software script, mathematical theory, and real-world use case, this guide leaves no stone unturned.

---

## 1. Executive Project Summary

### 1.1 The Vision and Paradigm Shift
Traditionally, ROS 2 SLAM robots follow a rigid architectural pattern: a low-level microcontroller (Arduino/Teensy) handles hardware PWM and encoders, while a high-level SBC (Raspberry Pi/Jetson) runs Ubuntu, ROS 2, and the SLAM algorithms, with the two communicating over a serial cable. 

This project shatters that paradigm. By utilizing **Micro-ROS** (the microcontroller port of ROS 2) and the **ESP32's dual-core architecture**, the robot communicates directly with a central workstation over UDP Wi-Fi using the XRCE-DDS protocol. The ESP32 is no longer just a dumb hardware driver; it is a native citizen of the ROS 2 graph. 

This drastically reduces the cost, weight, and power consumption of the robot. A standard Raspberry Pi 4 draws ~3-5 Watts of power continuously, requiring massive battery banks. The ESP32 draws a fraction of a Watt, extending battery life by magnitudes while still delivering real-time telemetry to the base station.

### 1.2 Core Technical Capabilities
* **Differential Drive Kinematics:** Custom translation of raw encoder ticks into $X, Y, \theta$ coordinate tracking.
* **Sensor Fusion (EKF):** Blending IMU angular velocity with wheel odometry to eliminate mechanical wheel slip drift.
* **Faux-LiDAR Generation:** Using a single-point Time-of-Flight (ToF) laser rangefinder to simulate a 1D `sensor_msgs/LaserScan` array, tricking the SLAM algorithms into mapping the environment.
* **Fully Distributed Processing:** The robot handles hardware, the workstation handles the heavy SLAM matrix calculations, and they sync via a 5GHz Wi-Fi network.
* **Dual-Core RTOS:** Leveraging FreeRTOS to pin hardware interrupts to Core 0 and heavy DDS serialization to Core 1, ensuring zero dropped encoder ticks.

---

## 2. Advanced Use Cases and Applications

A robot capable of SLAM and remote teleoperation is a generalized platform. Here are several real-world use cases for this specific architecture, and how the platform can be adapted for them.

### 2.1 Warehouse Automated Guided Vehicles (AGV)
**The Scenario:** Warehouses require robots to move inventory from shelves to packing stations. 
**Implementation:** By running the `nav2` (Navigation 2) stack on top of our SLAM map, the robot can be given a goal pose (`geometry_msgs/PoseStamped`). Nav2 will generate a global path around static shelves and use local planners (like DWB) to avoid dynamic obstacles (like humans).
**Hardware Modifications:** The single ToF sensor would need to be replaced by 3 ToF sensors (Left, Center, Right) or a cheap 2D LiDAR to provide a wider field of view for dynamic obstacle avoidance. The Rhino 20Kgcm motors are already powerful enough to drag a small cart.

### 2.2 Remote Inspection and Surveillance
**The Scenario:** Inspecting hazardous environments (crawl spaces, under houses, irradiated zones) where humans cannot safely enter.
**Implementation:** The robot is driven manually via `teleop_twist_keyboard` from a safe distance. Because the communication is over Wi-Fi, the operator can be anywhere on the local network. The SLAM algorithm builds a map of the unknown environment in real-time, ensuring the operator knows the exact layout of the hazardous zone.
**Hardware Modifications:** Adding an ESP32-CAM module to stream MJPEG video back to the ROS 2 host via HTTP, giving the operator first-person vision.

### 2.3 Educational Platform for ROS 2 and DDS
**The Scenario:** Universities teaching autonomous robotics currently rely on expensive platforms like the TurtleBot3 ($400+). 
**Implementation:** This architecture provides a $50 alternative. Students can learn the exact same concepts (TF trees, URDF, EKF, SLAM, Nav2, DDS Middleware) on a platform they built themselves. It teaches the critical embedded systems concept of offloading heavy compute to a base station, which is how actual planetary rovers operate.

### 2.4 Precision Agriculture Mapping
**The Scenario:** Mapping the layout of a greenhouse or small crop field.
**Implementation:** The robot navigates the rows between crops. The odometry tracks the distance, and the ToF sensor detects the presence of stalks or pots.
**Hardware Modifications:** The wheels must be swapped for larger, treaded tires. The PID values in the motor controller must be tuned to handle dirt/mud friction instead of flat hardwood floors.

---

## 3. Deep Dive: Hardware Architecture & Components

### 3.1 The Processing Core: ESP32 DevKit V1
The ESP32 was chosen not just for its built-in Wi-Fi, but for its FreeRTOS compatibility and dual Tensilica Xtensa LX6 microprocessors running at 240 MHz. Micro-ROS is highly memory-intensive because it must serialize DDS packets. The ESP32's 520 KB of SRAM is the absolute minimum requirement to maintain a stable connection while simultaneously handling high-frequency hardware interrupts. It also features hardware MACs for Wi-Fi, meaning the CPU is not bogged down by network layer processing.

### 3.2 The Motor Controller: Toshiba TB6612FNG
A dual H-bridge motor driver. 
* **The Theory of H-Bridges:** An H-bridge is an arrangement of four switches (transistors). By closing two diagonal switches, current flows through the motor in one direction. By closing the opposite two, current flows in reverse.
* **Why not the L298N?** The L298N uses ancient Bipolar Junction Transistors (BJTs) which drop nearly 2V of power as pure heat. The TB6612FNG uses MOSFETs, meaning it runs cool and delivers maximum voltage to the motors.
* **Control Scheme:** It requires three pins per motor. `IN1` and `IN2` control the H-bridge polarity (Forward/Reverse), and the `PWM` pin controls the switching frequency of the MOSFETs to regulate speed via the ESP32's LEDC hardware timer.

### 3.3 The Main Sensor: Waveshare ToF Mini Laser Range Sensor
* **Link:** [Waveshare ToF Mini Laser Range Sensor](https://robu.in/product/waveshare-tof-time-of-flight-mini-laser-range-sensor-uart-i2c-communication-support-long-range/?gad_source=1&gad_campaignid=20363337560&gclid=Cj0KCQjwsMLSBhD9ARIsAIpUTDoZb-_PtQL8MWx0B5zlwO2ZmK93kwW-OsbUg_i9DYIQdo_6Q3zwHPQaAm5yEALw_wcB)
* **The Physics of ToF:** Unlike cheap ultrasonic sensors (HC-SR04) which measure the speed of sound and suffer from wide, inaccurate cones and echo interference, this specific Waveshare module uses true Time-of-Flight laser technology. It emits an invisible 940nm laser pulse. A timer starts. When the photons bounce off an object and return, they strike a SPAD (Single Photon Avalanche Diode) array. The sensor calculates distance using $d = \frac{c \times t}{2}$ where $c$ is the speed of light.
* **Communication:** It supports both UART and I2C. We utilize the I2C interface to daisy-chain it with the IMU, saving GPIO pins on the ESP32. It provides millimeter-accurate distance readings up to several meters.

### 3.4 The Inertial Measurement Unit: HW-290 GY-87 10-DOF
* **Link:** [HW-290 GY-87 10DOF 3-Axis Gyro/Accel/Mag/Pressure](https://robokits.co.in/sensors/gyroscope-and-inertial-imu/hw-290-gy-87-10dof-3axis-gyro-acceleration-magnetometer-air-pressure-sensor?srsltid=AfmBOopDKzS7hymLZHUJeCKGROLV9weOwNYjxcgEDxiGuXyVWs92NUGs)
* **Description:** While basic projects use a standard MPU6050, we utilized the GY-87 10-DOF IMU module. This integrates an MPU6050 (3-axis gyro + 3-axis accelerometer), an HMC5883L (3-axis magnetometer for compass heading), and a BMP180 (barometric pressure sensor) all on a single board.
* **Purpose in SLAM:** When a robot turns on a carpet, the wheels might spin, but the robot might not actually rotate. Wheel encoders alone cannot detect this mechanical slip. The IMU's Z-axis gyroscope physically measures the actual angular velocity ($\omega_z$) of the chassis using micro-electromechanical (MEMS) tuning forks. By fusing this raw physics data with the mathematical wheel odometry inside our Extended Kalman Filter, we completely eliminate rotational drift.

### 3.5 The Motors: Rhino GB37 12V DC Geared Encoder Motors
* **Link:** [Rhino GB37 12V 20RPM 20Kgcm Geared Motor](https://robokits.co.in/motors/rhino-gb37-12v-dc-geared-motor/dc-12v-encoder-servo-motors/rhino-gb37-12v-20rpm-20kgcm-dc-geared-encoder-servo-motor?srsltid=AfmBOoqEhGNUj4FYQkDmy4y3DXXwsSW6rBORmcCZZuFRJ8cW6nGZuLDi)
* **Mechanical Power:** Standard hobby TT motors are weak, lack feedback, and cannot carry batteries. We upgraded to heavy-duty, industrial-grade Rhino GB37 12V DC gear motors. They provide a massive 20Kgcm of stall torque through a precision metal gearbox, easily carrying a heavy robot chassis loaded with 12V batteries. 
* **Quadrature Decoding:** Attached to the rear of these motors are ultra-high-resolution magnetic quadrature encoders. As the motor spins, magnets pass over dual Hall-effect sensors, generating two perfectly synced square waves (Phase A and Phase B). Because the two sensors are physically offset by 90 degrees, we can determine absolute direction: if Phase A goes HIGH while Phase B is LOW, the motor is spinning forward. If Phase A goes HIGH while Phase B is HIGH, it is spinning backward.

### 3.6 Power Delivery & EMF Isolation
Power isolation is critical in robotics to prevent Electromagnetic Force (EMF) noise from crashing the sensitive microcontroller.
* **Logic Power (3.3V):** The ESP32 is powered via USB (5V), which passes through an internal Low-Dropout (LDO) regulator to produce clean 3.3V. This 3.3V feeds the I2C sensors and the TB6612FNG logic pins.
* **Motor Power (12V):** A separate 12V LiPo battery is connected *exclusively* to the `VMOT` pin on the motor driver. When the DC motors start, they draw massive spike currents (inrush current). If the ESP32 shared this power rail without massive capacitors, the voltage drop would cause a brown-out reset. 
* **Grounding:** The grounds of the 12V battery and the ESP32 must be connected together to provide a common reference voltage, but the positive voltages remain strictly isolated.

---

## 4. Comprehensive Wiring & Pinout Matrix

To replicate this exact build, the wiring must be flawless. Here is the definitive pinout matrix mapping the logical systems to the physical ESP32 pins.

| Subsystem | Component | Pin Name | ESP32 GPIO | Description / Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Sensor Bus** | **Waveshare ToF** | SDA | GPIO 32 | I2C Data Line (Requires Pull-up) |
| **Sensor Bus** | **Waveshare ToF** | SCL | GPIO 33 | I2C Clock Line (Requires Pull-up) |
| **Sensor Bus** | **Waveshare ToF** | VCC | 3.3V | Logic Power |
| **Sensor Bus** | **Waveshare ToF** | GND | GND | Common Ground |
| **Sensor Bus** | **GY-87 IMU** | SDA | GPIO 32 | Shared I2C Data Line |
| **Sensor Bus** | **GY-87 IMU** | SCL | GPIO 33 | Shared I2C Clock Line |
| **Sensor Bus** | **GY-87 IMU** | VCC | 3.3V | Logic Power |
| **Sensor Bus** | **GY-87 IMU** | GND | GND | Common Ground |
| **Motor Control**| **TB6612FNG** | PWMA | GPIO 19 | Left Motor Speed (LEDC Timer) |
| **Motor Control**| **TB6612FNG** | AIN1 | GPIO 18 | Left Motor Direction 1 |
| **Motor Control**| **TB6612FNG** | AIN2 | GPIO 5 | Left Motor Direction 2 |
| **Motor Control**| **TB6612FNG** | PWMB | GPIO 23 | Right Motor Speed (LEDC Timer) |
| **Motor Control**| **TB6612FNG** | BIN1 | GPIO 21 | Right Motor Direction 1 |
| **Motor Control**| **TB6612FNG** | BIN2 | GPIO 22 | Right Motor Direction 2 |
| **Motor Control**| **TB6612FNG** | STBY | GPIO 4 | Standby (Must be driven HIGH 3.3V) |
| **Motor Control**| **TB6612FNG** | VCC | 3.3V | Logic Power for the H-Bridge Gates |
| **Motor Control**| **TB6612FNG** | VMOT | 12V BAT | High-current power for Rhino Motors |
| **Odometry** | **Left Encoder** | Phase A | GPIO 14 | Interrupt Pin (RISING Edge) |
| **Odometry** | **Left Encoder** | Phase B | GPIO 13 | Direction Polling Pin |
| **Odometry** | **Right Encoder**| Phase A | GPIO 27 | Interrupt Pin (RISING Edge) |
| **Odometry** | **Right Encoder**| Phase B | GPIO 26 | Direction Polling Pin |

---

## 5. Firmware Architecture: The ESP-IDF C Code (`main.c`)

Writing stable embedded C code for Micro-ROS requires strict adherence to memory management and real-time operating system (RTOS) principles. Standard Arduino loops are insufficient for high-speed robotic control.

### 5.1 The Dual-Core FreeRTOS Strategy
The ESP32 is unique among cheap microcontrollers because it has two physical Tensilica CPU cores. If a single core was forced to handle I2C blocking calls, Wi-Fi interrupts, and Micro-ROS serialization simultaneously, the system would instantly crash with watchdog timeouts. We explicitly partitioned the firmware:

#### Core 0: The Hardware Core (The Physical Realm)
Core 0 is dedicated to the physical world. It runs three FreeRTOS tasks:
1. `encoder_reader_task`: Highest priority. Handles the hardware interrupts triggered by the wheel encoders.
2. `imu_reader_task`: Polls the GY-87 registers over I2C at 50Hz.
3. `tof_reader_task`: Polls the Waveshare ToF sensor at 20Hz.

#### Core 1: The Network Core (The Digital Realm)
Core 1 is entirely dedicated to the ROS 2 graph. It runs a single massive task:
1. `micro_ros_task`: This task allocates the XRCE-DDS memory pool, establishes the UDP connection to the Micro-ROS Agent, and enters an infinite `rclc_executor_spin_some` loop. It consumes the global variables updated by Core 0 (protected by Mutexes to prevent race conditions) and publishes them to the Wi-Fi network.

### 5.2 Handling Encoder Interrupts in C
Encoders on the Rhino motors spin extremely fast. If we polled them in a standard `while(1)` loop, we would miss ticks, causing the SLAM map to skew horribly. We use the ESP-IDF GPIO ISR (Interrupt Service Routine) service:
```c
static void IRAM_ATTR left_enc_isr_handler(void* arg) {
    if (gpio_get_level(PIN_ENC_L_B)) {
        left_ticks++;
    } else {
        left_ticks--;
    }
}
```
**The Secret to Stability:** The `IRAM_ATTR` flag is crucial. It forces the compiler to place this specific function into the ESP32's internal RAM rather than the external flash memory. Accessing flash takes milliseconds; accessing IRAM takes microseconds. This ensures the interrupt executes instantly, preventing CPU locking when the wheels are spinning at maximum RPM.

### 5.3 Motor Control (PWM Generation via LEDC)
To drive the motors, we use the ESP32's LEDC (LED Control) hardware peripheral, which is a highly accurate hardware timer that doesn't consume CPU cycles like software PWM (`delayMicroseconds`).
```c
// Mapping -1.0 to 1.0 into 12-bit PWM (400 to 4095)
uint32_t pwl = (uint32_t)fmap(fabs(left), 0.0f, 1.0f, 400.0f, 4095.0f);
```
**The Deadband Problem:** We clamp the minimum duty cycle at `400` because physical DC gearboxes have static friction (stiction). If we set the duty cycle to `100` (out of 4095), the motor coils would energize and emit a high-pitched whine, but the torque would be insufficient to physically overcome the gears. `400` is our measured deadband threshold where movement begins.

### 5.4 The XRCE-DDS Middleware (Micro-ROS)
Micro-ROS does not use the heavy FastDDS or CycloneDDS implementations found in full ROS 2 Ubuntu distributions. It uses eProsima's **Micro XRCE-DDS**. 
The ESP32 acts as a *Client*. It serializes the ROS 2 messages into a highly compact binary format and wraps them in UDP packets. The Micro-ROS Agent on the Ubuntu host acts as the *Server*. The Agent receives these UDP packets, deserializes them, and injects them into the standard ROS 2 FastDDS network. This bridging is what makes Micro-ROS so powerful—it offloads the heavy DDS discovery protocol (which uses multicast and massive XML files) to the host machine, keeping the ESP32 lean.

**The ESP32 Entities:**
* **Node:** `esp32_robot`
* **Publishers:** `/left_ticks`, `/right_ticks`, `/tof_distance`, `/imu/data`
* **Subscribers:** `/cmd_vel`

---

## 6. Host Architecture: The ROS 2 Python Nodes

Once the raw data (`left_ticks`, `right_ticks`, `tof_distance`, `imu/data`) arrives on the Ubuntu 22.04 host, our custom ROS 2 node graph takes over. We chose Python for the host-side nodes for rapid prototyping, as performance on an i7 laptop is virtually unlimited compared to the ESP32.

### 6.1 The Kinematics Engine (`wheel_odom_node.py`)
Raw ticks mean nothing to a SLAM algorithm. We must convert them into a `nav_msgs/Odometry` message. The Python node implements standard Differential Drive Kinematics.

**The Mathematics & Python Implementation:**
```python
# 1. Calculate distance per tick based on physical constants
DISTANCE_PER_TICK = (math.pi * WHEEL_DIAMETER_METERS) / TICKS_PER_REVOLUTION

# 2. Calculate distance traveled by each wheel since last loop
d_left = delta_ticks_left * DISTANCE_PER_TICK
d_right = delta_ticks_right * DISTANCE_PER_TICK

# 3. Calculate Center Linear Velocity (V) and Angular Velocity (W)
v = (d_left + d_right) / (2.0 * dt)
w = (d_right - d_left) / (WHEEL_BASE_METERS * dt)

# 4. Integrate Velocities into Absolute Position (Pose)
theta += w * dt
x += v * math.cos(theta) * dt
y += v * math.sin(theta) * dt
```
The node packs $x, y, \theta$ into the `Pose` covariance matrix, and $v, w$ into the `Twist` covariance matrix, and publishes to `/wheel/odometry`.

### 6.2 Sensor Fusion (`ekf_node` from `robot_localization`)
Because the Rhino motors slip on hardwood floors or carpets, the mathematically calculated $\theta$ from `wheel_odom_node.py` will slowly drift. Over 10 meters, a 2-degree drift will cause SLAM to build twisted, useless maps.
We use an Extended Kalman Filter (EKF) to fuse the calculated odometry with the physical GY-87 gyroscope data.

**The `ekf.yaml` Matrix Configuration:**
```yaml
odom0: /wheel/odometry
odom0_config: [true,  true,  false,  # Trust Odom X, Y
               false, false, true,   # Trust Odom Yaw
               true,  false, false,  # Trust Odom dX
               false, false, true,   # Trust Odom dYaw
               false, false, false]

imu0: /imu/data
imu0_config: [false, false, false,
              false, false, true,    # Heavily Trust IMU Yaw
              false, false, false,
              false, false, true,    # Heavily Trust IMU dYaw
              true,  false, false]
```
The Kalman Filter is a probabilistic algorithm. It looks at the variance of the wheels, looks at the variance of the IMU, and mathematically determines the "most likely" true heading of the robot. The output is a highly stabilized TF broadcast mapping the `odom` frame to the `base_footprint` frame.

### 6.3 The Faux-LiDAR (`tof_to_laserscan_realtime.py`)
`slam_toolbox` expects a 360-degree rotating LiDAR (like an RPLiDAR A1). We only have a single Waveshare ToF sensor pointing straight ahead. 
To trick SLAM into working on a budget, we generate a fake `sensor_msgs/LaserScan` message. 

**The Trick:**
1. We define a highly narrow field of view: `angle_min = -0.026 rad`, `angle_max = 0.026 rad` (about 3 degrees).
2. We create a `ranges` array with exactly 3 elements.
3. We populate all 3 elements with the distance value received from `/tof_distance` (converted from mm to meters).
4. We set the `frame_id` to `laser_frame`.

While this only allows the robot to map a tiny dot directly in front of it, spinning the robot in place allows the SLAM algorithm to slowly "paint" a 2D map of the room by sweeping the laser dot across the walls, exactly like a radar dish.

### 6.4 Mapping (`slam_toolbox`)
We utilize the `async_slam_toolbox_node`. 
* **Input:** The `scan` topic and the `tf` tree.
* **Mechanism:** It uses scan matching. It takes the current laser hits, looks at the previous laser hits, and nudges the robot's assumed position until the hits align perfectly. This corrects minor odometry errors. It then updates a global 2D Occupancy Grid.
* **Output:** The `/map` topic, which is visualized as the gray (unknown), white (free space), and black (wall) grid in RViz.

### 6.5 Teleop & RViz Visualizer
* **`teleop_twist_keyboard`:** A standard ROS 2 utility. It captures WASD/IJKL keys, translates them into `geometry_msgs/Twist` messages, and publishes them to `/cmd_vel`. The ESP32 subscribes to this and executes the movement.
* **`rviz2`:** The visualization GUI. It loads the `tof_robot.urdf.xacro` file to render a 3D box representing the robot. It overlays the `/map` grid, and visualizes the faux-LiDAR hits as red dots.

---

## 7. The Launch System & Bash Automation

Managing 6 different ROS 2 nodes, an EKF configuration file, an RViz configuration file, and a Micro-ROS UDP agent manually requires opening 7 terminal windows and typing flawlessly. To solve this, we created a robust, automated launch infrastructure.

### 7.1 `robot_complete.launch.py`
A Python launch file that starts all the host-side nodes simultaneously. 
* It uses the `Command` substitution to dynamically parse the `xacro` URDF file and pass it to `robot_state_publisher`.
* It loads the parameters from `ekf.yaml` into the `robot_localization` node.
* It launches `rviz2` with a pre-configured `.rviz` perspective file, so the user doesn't have to manually add the Map and LaserScan displays every time.

### 7.2 `run_robot.sh`
The master bash orchestration script. It handles the chaotic reality of Linux process management.
1. **The Nuclear Cleanup:** It violently kills any existing ROS processes (`killall -9`) to prevent zombie nodes from corrupting the TF tree.
2. **Agent Deployment:** It launches the `micro-ros-agent udp4 --port 8888` in the background and prompts the user to physically reset the ESP32 to trigger the DDS handshake.
3. **Domain Synchronization:** It forces `export ROS_DOMAIN_ID=0` so the host matches the agent's DDS partition.
4. **Execution:** It calls the ROS 2 launch files, piping their verbose output to `/tmp/robot_log.txt` to keep the user's terminal clean and beautiful.

---

## 8. Setup, Configuration & Deployment Guide

To replicate this project from scratch, follow these exact terminal commands.

### 8.1 ESP-IDF Configuration (`menuconfig`)
Before building the C code, the ESP32 firmware must be injected with your local network credentials.
```bash
# Source the ESP-IDF v5.1 environment
. ~/esp/esp-idf-v5.1/export.sh
cd ~/tof_microros_project

# Open the configuration GUI
idf.py menuconfig
```
1. Navigate to `micro-ROS Configuration`.
2. Select `WiFi Configuration`. Enter your router's SSID and Password.
3. Select `micro-ROS Agent IP`. Enter the static IP address of the laptop running Ubuntu (e.g., `10.42.0.1`).
4. Ensure the Port is set to `8888`.
5. Press `S` to Save, and `Q` to Quit.

### 8.2 Flashing the ESP32 DevKit
```bash
# Clean old configurations, build, flash, and open serial monitor
idf.py fullclean
idf.py build flash monitor
```
**Hardware Quirk:** ESP32 DevKits sometimes fail to transition into download mode automatically via the CH340 chip. If the terminal prints `Connecting........_` and stalls, you must physically press and hold the `BOOT` button on the ESP32 board until the flashing percentage begins.

### 8.3 ROS 2 Workspace Build
```bash
cd ~/tof_microros_project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 8.4 Execution
Turn on the 12V battery, then run:
```bash
cd ~/tof_microros_project
./run_robot.sh
```
Open a new terminal for driving:
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 9. The Great Debugging Journey (Problems & Solutions)

Building a complex robotics stack from scratch is never a linear process. It requires methodical isolation of variables. Here is a massive breakdown of the deepest, most complex bugs we faced during development, and the exact diagnostic processes used to eradicate them.

### Phase 1: The Docker Network Isolation
**The Problem:** The ESP32 code compiled perfectly and booted. It connected to the Wi-Fi and acquired an IP (`10.42.0.49`). However, it stalled forever on "Waiting for agent...". We were running the Micro-ROS agent inside a Docker container via `docker run -p 8888:8888/udp microros/micro-ros-agent`.
**The Diagnosis:** Docker, by default, places containers on their own isolated network bridge (`172.17.x.x`). While port forwarding usually works for TCP, UDP discovery packets for XRCE-DDS were getting mangled across the subnet boundary. The ESP32's UDP packets were reaching the host, but the host didn't know how to route the UDP response back to the ESP32's `10.42.x.x` subnet.
**The Solution:** We abandoned the Docker approach. We installed the native Snap version of the agent (`sudo snap install micro-ros-agent`). Because it runs natively on the host OS, it binds directly to the `10.42.0.1` interface. The ESP32 connected instantly.

### Phase 2: The Silent Domain Mismatch
**The Problem:** The ESP32 connected! The serial monitor proudly declared `micro-ROS connected!`. However, when we ran `ros2 topic list` on the host laptop, the topics (`/left_ticks`, `/cmd_vel`) were completely missing.
**The Diagnosis:** We assumed the agent was broken. We checked the agent logs, and it showed the ESP32 successfully creating data writers. Why couldn't ROS 2 see them? We realized that the laptop's `~/.bashrc` had a line: `export ROS_DOMAIN_ID=42` from a previous project. ROS 2 isolates networks using Domain IDs. The Snap micro-ros-agent defaults to Domain `0`. The ESP32 was publishing data to Domain 0, but the laptop's tools were only listening to Domain 42. It was like two walkie-talkies on different channels.
**The Solution:** We updated our terminal environment and our automated `run_robot.sh` script to explicitly run `export ROS_DOMAIN_ID=0` before launching *anything*. Suddenly, `ros2 topic list` lit up with all our ESP32 topics.

### Phase 3: The Obstacle Avoidance Watchdog Fight
**The Problem:** We opened `teleop_twist_keyboard` and pressed `i` to drive forward. The robot model in RViz didn't move. We ran `ros2 topic echo /cmd_vel` and saw that it was being spammed with `linear.x = 0.0`. But teleop was sending `0.5`!
**The Diagnosis:** We used the command `ros2 topic info -v /cmd_vel`. This verbose command reveals exactly *who* is publishing to a topic. We discovered two publishers: `teleop_twist_keyboard` AND `obstacle_avoidance.py`. 
We inspected the Python code for `obstacle_avoidance.py` and found a safety watchdog function. If this node didn't receive a fresh teleop command every 0.5 seconds, it automatically published a zero-velocity `Twist` message to stop the robot. Because we were routing teleop to `/cmd_vel` directly instead of `/cmd_vel_raw`, the watchdog was starving and actively fighting our keyboard commands, flooding the ESP32 with stop commands.
**The Solution:** We opened `robot_complete.launch.py` and completely removed the `obstacle_avoidance` node from the launch sequence. This bypassed the safety gate, allowing `teleop` to be the sole, undisputed publisher to `/cmd_vel`.

### Phase 4: The "Dancing" RViz Ghost Nodes
**The Problem:** With the data flowing, we opened RViz. The robot model appeared on the screen, but it was violently flickering. The 3D model was rapidly teleporting back and forth between two completely different locations on the grid at 60 frames per second.
**The Diagnosis:** Flickering in RViz is the hallmark symptom of a "TF Conflict"—when two different programs are trying to tell RViz where the robot is at the same time. We ran `tf2_monitor` and `ros2 topic info -v /tf`. The output was massive. We saw *four* separate instances of `slam_toolbox` and *two* separate instances of `ekf_node` actively publishing to the `/tf` tree.
**The Root Cause:** Linux process management. Earlier in the session, we had pressed `Ctrl+C` in the terminal to kill the launch script. However, because the nodes were spawned as background Python processes via the ROS 2 launch system, they didn't receive the termination signal cleanly. They became "zombie" processes. They continued to run silently in the background of the operating system. When we ran the launch script a second time, it spawned a *new* set of nodes. The new nodes and the zombie nodes were now fighting in a brutal tug-of-war over the robot's mathematical location.
**The Eradication:** We realized that `Ctrl+C` was untrustworthy. We wrote a nuclear bash command:
`killall -9 robot_state_publisher ekf_node async_slam_toolbox_node teleop_twist_keyboard rviz2 micro-ros-agent`
The `-9` flag sends a `SIGKILL` directly to the Linux kernel, forcefully terminating the processes without allowing them to block the signal. We placed this command at the very top of `run_robot.sh`. Now, every single time the user launches the robot, the script aggressively sweeps the operating system for zombie nodes and annihilates them, ensuring a perfectly clean, flicker-free TF tree.

### Phase 5: The Hardware Power Illusion
**The Problem:** The software was absolutely flawless. We ran a test by typing `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"`. We watched the ESP32's serial monitor over USB, and it printed: `CMD_VEL received: lin=1.00 ang=0.00`. The code was successfully mapping this to PWM and pushing it to the `motors_set()` function. The logic was flawless. But the physical TB6612FNG motor driver sitting on the desk was completely inert. The wheels would not spin.
**The Diagnosis:** We were paranoid that the FreeRTOS dual-core setup was causing the PWM driver to crash. We reviewed the ESP32 initialization sequence. On boot, the ESP32 runs a 1-second `test_motors()` function. We asked ourselves: *Did the motors spin during the boot test?* 
The answer was no. 
We realized the ESP32 was plugged into the laptop via USB for serial debugging. USB provides 5V. This 5V goes to the ESP32, which steps it down to 3.3V. This 3.3V power rail was feeding the ESP32 CPU, the Wi-Fi antenna, the ToF sensor, the IMU, the wheel encoders, and the logic-level pins (`VCC`) of the TB6612FNG motor driver.
**The Root Cause:** Physics. A standard DC gear motor requires significant current (Amps) and voltage to physically turn the copper coils inside the magnetic stator. The motor driver has a dedicated pin called `VMOT` (Voltage Motor) specifically for this high-current power. The `VMOT` pin was disconnected. We were asking the motor driver to spin the wheels, but we hadn't given it any electricity to do so. The USB 5V rail simply cannot provide enough power, and attempting to route motor current through the ESP32 would instantly fry the microcontroller.
**The Eradication:** We unplugged the USB cable, terminating the serial monitor. We plugged a 12V LiPo battery directly into the `VMOT` and `GND` pins of the motor chassis and flipped the power switch. The ESP32 booted via the 5V BEC (Battery Eliminator Circuit) from the battery. It connected to the Micro-ROS agent over Wi-Fi. We opened the teleop terminal and pressed `i`. The wheels violently roared to life, spinning at 50% duty cycle. The hardware encoders began firing thousands of interrupts. The ESP32 transmitted the ticks to the host. The `wheel_odom_node` did the kinematics math and generated odometry. The `ekf_node` fused it with the IMU. And the robot model in RViz began mapping the room.

### Phase 6: The Inverted Motor Kinematics
**The Problem:** The robot was driving, but the controls were completely mangled. Pressing `i` (Forward) made the robot move backward. Pressing `,` (Backward) made it move forward. Pressing `j` (Turn Left) made it rotate right, and pressing `l` (Turn Right) made it rotate left.
**The Diagnosis:** This specific pattern—where every linear and angular vector is perfectly inverted—is mathematical proof that **both** physical DC gear motors were wired backwards relative to the chassis's "front" vector. When the software commanded a positive PWM duty cycle to drive the wheels "forward", the physical electromagnetic coils were energized in reverse, spinning the wheels backward.
**The Solution:** Rather than disassembling the robot chassis and desoldering the motor terminals, we resolved the hardware flaw purely in software. We modified `main.c` and inverted the H-Bridge logic inside the `motors_set()` function.
Standard forward logic for the TB6612FNG is `IN1=1, IN2=0`. We explicitly inverted this to `IN1=0, IN2=1`. When the ESP32 received a forward `cmd_vel`, it commanded the H-Bridge in "reverse", which perfectly counteracted the physical backward wiring, causing the robot to drive precisely forward.

---

## 10. Future Expandability & Upgrades
Because the architecture relies heavily on standard ROS 2 message types (`Twist`, `LaserScan`, `Odometry`), the hardware is entirely decoupled from the high-level intelligence. The base station doesn't care that an ESP32 is generating the LaserScan; it just processes standard ROS 2 data.
* **Nav2 Integration:** The next step is to integrate the `nav2` stack. Because the `cmd_vel` and `odom` topics are fully standard, Nav2 can be dropped in with zero changes to the ESP32 firmware, allowing the robot to autonomously pathfind around the map it just created.
* **True LiDAR:** The `tof_to_laserscan` python node can be trivially replaced with a real RPLiDAR A1 node, instantly upgrading the robot from a 3-degree cone of vision to a 360-degree high-resolution map, again, with zero changes to the microcontroller firmware.
* **Camera Streaming:** Attaching a secondary ESP32-CAM to the chassis allows for a separate HTTP MJPEG stream, enabling first-person view (FPV) teleoperation without clogging the Micro-ROS XRCE-DDS UDP bandwidth.
