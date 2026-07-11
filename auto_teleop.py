#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AutoTeleop(Node):
    def __init__(self):
        super().__init__('auto_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = 0
        self.state_start_time = time.time()
        
        self.get_logger().info('Autonomous Teleop started! Driving in a square.')

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        elapsed = current_time - self.state_start_time
        
        # State 0: Move Forward for 2 seconds
        if self.state == 0:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            if elapsed > 2.0:
                self.state = 1
                self.state_start_time = current_time
                self.get_logger().info('Turning...')
        
        # State 1: Turn Left for 1.5 seconds (approx 90 deg)
        elif self.state == 1:
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            if elapsed > 1.5:
                self.state = 0
                self.state_start_time = current_time
                self.get_logger().info('Moving forward...')
                
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    auto_teleop = AutoTeleop()
    try:
        rclpy.spin(auto_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on exit
        if rclpy.ok():
            stop_msg = Twist()
            auto_teleop.publisher_.publish(stop_msg)
            auto_teleop.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
