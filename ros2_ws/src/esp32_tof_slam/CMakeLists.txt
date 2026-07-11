cmake_minimum_required(VERSION 3.8)
project(esp32_tof_slam)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  src/wheel_odom_node.py
  src/tof_to_laserscan_realtime.py
  src/obstacle_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
