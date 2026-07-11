#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu, LaserScan
import math

GYRO_DZ = 0.008


class ToFLaserScan(Node):
    def __init__(self):
        super().__init__("tof_to_laserscan")
        self.N         = 360
        self.RANGE_MIN = 0.05
        self.RANGE_MAX = 4.00
        self.DECAY_S   = 30.0

        self.ranges     = [self.RANGE_MAX] * self.N
        self.timestamps = [0.0]            * self.N

        self.yaw        = 0.0
        self.yaw_init   = None
        self.last_imu_t = None

        self.left_prev  = None
        self.right_prev = None
        self.left_cur   = 0
        self.right_cur  = 0

        self.tof_m = self.RANGE_MAX

        self.create_subscription(Int32, "/tof_distance", self.tof_cb,   10)
        self.create_subscription(Imu,   "/imu/data",     self.imu_cb,   10)
        self.create_subscription(Int32, "/left_ticks",   self.left_cb,  10)
        self.create_subscription(Int32, "/right_ticks",  self.right_cb, 10)

        self.pub = self.create_publisher(LaserScan, "/scan", 5)

        # Publish at 5Hz — prevents SLAM queue overflow
        self.create_timer(0.2, self.publish_scan)

        self.get_logger().info("TOF LaserScan ready at 5Hz (motion-gated)")

    def left_cb(self, msg):
        if self.left_prev is None: self.left_prev = msg.data
        self.left_cur = msg.data

    def right_cb(self, msg):
        if self.right_prev is None: self.right_prev = msg.data
        self.right_cur = msg.data

    def tof_cb(self, msg):
        d = msg.data / 1000.0
        self.tof_m = max(self.RANGE_MIN, min(self.RANGE_MAX, d))

    def imu_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_imu_t is None:
            self.last_imu_t = now
            return
        dt = now - self.last_imu_t
        self.last_imu_t = now

        gz = msg.angular_velocity.z
        gz = gz if abs(gz) > GYRO_DZ else 0.0
        self.yaw += gz * dt

        if self.yaw_init is None:
            self.yaw_init = self.yaw

        if self.left_prev is None or self.right_prev is None:
            return

        # MOTION GATE: only update scan angle when wheels are moving
        dl = abs(self.left_cur  - self.left_prev)
        dr = abs(self.right_cur - self.right_prev)
        self.left_prev  = self.left_cur
        self.right_prev = self.right_cur
        if dl == 0 and dr == 0:
            return  # Robot stationary — freeze scan angles

        rel_yaw   = self.yaw - self.yaw_init
        angle_deg = math.degrees(rel_yaw) % 360.0
        idx       = int(round(angle_deg * self.N / 360.0)) % self.N
        ts        = now

        for off in range(-3, 4):
            i = (idx + off) % self.N
            self.ranges[i]     = self.tof_m
            self.timestamps[i] = ts

    def publish_scan(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        out = [self.ranges[i] if (now - self.timestamps[i]) < self.DECAY_S
               else self.RANGE_MAX for i in range(self.N)]

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        msg.angle_min       = 0.0
        msg.angle_max       = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / self.N
        msg.time_increment  = 0.0
        msg.scan_time       = 0.2
        msg.range_min       = self.RANGE_MIN
        msg.range_max       = self.RANGE_MAX
        msg.ranges          = out
        msg.intensities     = [100.0] * self.N
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ToFLaserScan()
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
