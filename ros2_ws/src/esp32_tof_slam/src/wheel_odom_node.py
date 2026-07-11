#!/usr/bin/env python3
"""
Wheel Odometry Node - Single Timer (No Race Condition)
- Publishes /joint_states at 20Hz ALWAYS (fixes "No transform" in RViz)
- Publishes /wheel/odometry at 20Hz (consumed by EKF)
- Only integrates position when encoders actually tick (no drift)

FIX: right encoder is physically inverted (counts down on forward motion),
     so right_cur is negated to match ROS convention (positive = forward).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
import math

ENCODER_CPR   = 1320
WHEEL_RADIUS  = 0.035
WHEEL_SEP     = 0.175
DIST_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / ENCODER_CPR
GYRO_DZ       = 0.008


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__("wheel_odom_node")

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_prev   = None
        self.right_prev  = None
        self.left_cur    = 0
        self.right_cur   = 0
        self.left_angle  = 0.0
        self.right_angle = 0.0
        self.left_vel    = 0.0
        self.right_vel   = 0.0

        self.imu_gz = 0.0
        self.last_t = None

        self.odom_pub  = self.create_publisher(Odometry,   "/wheel/odometry", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states",   10)

        self.create_subscription(Int32, "/left_ticks",  self.left_cb,  10)
        self.create_subscription(Int32, "/right_ticks", self.right_cb, 10)
        self.create_subscription(Imu,   "/imu/data",    self.imu_cb,   10)

        self.create_timer(0.05, self.update)

        self.get_logger().info(
            "WheelOdomNode ready | CPR=%d | dist/tick=%.6fm | 20Hz" %
            (ENCODER_CPR, DIST_PER_TICK))

    def left_cb(self, msg):
        # Left encoder counts UP when left wheel moves forward — correct as-is
        self.left_cur = msg.data

    def right_cb(self, msg):
        # ✅ FIX: Right encoder is physically wired in reverse —
        # it counts DOWN when the right wheel moves forward.
        # Negate so that forward motion = positive ticks on both sides.
        self.right_cur = -msg.data

    def imu_cb(self, msg):
        gz = msg.angular_velocity.z
        self.imu_gz = gz if abs(gz) > GYRO_DZ else 0.0

    def update(self):
        now   = self.get_clock().now()
        stamp = now.to_msg()

        if self.last_t is None:
            self.last_t     = now
            self.left_prev  = self.left_cur
            self.right_prev = self.right_cur
            self._pub_joints(stamp)
            self._pub_odom(stamp, 0.0, 0.0)
            return

        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0:
            self._pub_joints(stamp)
            return

        dl_t = self.left_cur  - self.left_prev
        dr_t = self.right_cur - self.right_prev
        self.left_prev  = self.left_cur
        self.right_prev = self.right_cur

        if dl_t != 0 or dr_t != 0:
            d_l = dl_t * DIST_PER_TICK
            d_r = dr_t * DIST_PER_TICK
            d_c = (d_l + d_r) / 2.0

            d_theta = (0.6 * (d_r - d_l) / WHEEL_SEP) + (0.4 * self.imu_gz * dt)

            self.theta       += d_theta
            self.x           += d_c * math.cos(self.theta)
            self.y           += d_c * math.sin(self.theta)
            self.left_angle  += d_l / WHEEL_RADIUS
            self.right_angle += d_r / WHEEL_RADIUS
            self.left_vel     = (d_l / WHEEL_RADIUS) / dt
            self.right_vel    = (d_r / WHEEL_RADIUS) / dt
            v_lin = d_c     / dt
            v_ang = d_theta / dt
        else:
            self.left_vel  = 0.0
            self.right_vel = 0.0
            v_lin = 0.0
            v_ang = 0.0

        self._pub_joints(stamp)
        self._pub_odom(stamp, v_lin, v_ang)

    def _pub_joints(self, stamp):
        js = JointState()
        js.header.stamp = stamp
        js.name         = ["left_wheel_joint", "right_wheel_joint"]
        js.position     = [self.left_angle,  self.right_angle]
        js.velocity     = [self.left_vel,    self.right_vel]
        js.effort       = [0.0, 0.0]
        self.joint_pub.publish(js)

    def _pub_odom(self, stamp, v_lin, v_ang):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        o  = Odometry()
        o.header.stamp            = stamp
        o.header.frame_id         = "odom"
        o.child_frame_id          = "base_footprint"
        o.pose.pose.position.x    = self.x
        o.pose.pose.position.y    = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x    = v_lin
        o.twist.twist.angular.z   = v_ang
        o.pose.covariance[0]  = 0.005
        o.pose.covariance[7]  = 0.005
        o.pose.covariance[35] = 0.001
        o.twist.covariance[0]  = 0.005
        o.twist.covariance[35] = 0.001
        self.odom_pub.publish(o)


def main():
    rclpy.init()
    node = WheelOdomNode()
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

