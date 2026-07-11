#!/usr/bin/env python3
"""
Obstacle Avoidance Node
========================
Subscribes: /cmd_vel_raw   (teleop input, geometry_msgs/Twist)
            /tof_distance  (std_msgs/Int32, mm from ESP32)
Publishes:  /cmd_vel       (safe output to ESP32 robot)

Rules:
  - Forward + TOF < 150mm  -> STOP (full block)
  - Forward + TOF < 350mm  -> SLOW (50% speed)
  - Backward / turning     -> always pass through
  - No teleop for 0.5s     -> publish zero velocity (safety watchdog)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
STOP_DIST_MM = 150   # mm - hard stop
SLOW_DIST_MM = 350   # mm - slow to 50%
SLOW_FACTOR  = 0.5
WATCHDOG_S   = 2.0   # seconds


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")
        self.tof_mm       = 9999
        self.last_cmd_t   = self.get_clock().now()
        self.last_cmd     = Twist()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(Twist, "/cmd_vel_raw",    self.cmd_cb, 10)
        self.create_subscription(Int32, "/tof_distance",   self.tof_cb, 10)

        self.create_timer(0.05, self.watchdog)  # 20 Hz safety watchdog
        self.get_logger().info(
            "ObstacleAvoidance ready  STOP=%dmm SLOW=%dmm" %
            (STOP_DIST_MM, SLOW_DIST_MM))

    def tof_cb(self, msg):
        if msg.data <= 0:
            self.tof_mm = 9999  # 0 means out of range or error, don't halt the robot!
        else:
            self.tof_mm = msg.data

    def cmd_cb(self, msg):
        self.last_cmd_t = self.get_clock().now()
        out = Twist()
        lin = msg.linear.x
        ang = msg.angular.z

        # Only gate forward motion
        if lin > 0.0:
            if self.tof_mm < STOP_DIST_MM:
                lin = 0.0
                self.get_logger().warn(
                    "OBSTACLE! TOF=%dmm < %dmm STOPPED" %
                    (self.tof_mm, STOP_DIST_MM))
            elif self.tof_mm < SLOW_DIST_MM:
                lin *= SLOW_FACTOR
                self.get_logger().info(
                    "SLOW: TOF=%dmm" % self.tof_mm)

        out.linear.x  = lin
        out.angular.z = ang
        self.cmd_pub.publish(out)

    def watchdog(self):
        # Publish zero if no teleop for WATCHDOG_S seconds
        age = (self.get_clock().now() - self.last_cmd_t).nanoseconds * 1e-9
        if age > WATCHDOG_S:
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
