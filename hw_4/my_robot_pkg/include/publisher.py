#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
 
class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.callback)

        self.linear_speed = 2.0
        self.angular_amplitude = 2.0
        self.angular_frequency = 0.8
        self.start_time = self.get_clock().now()

    def callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.start_time).nanoseconds / 1e9

        msg = Twist()
        msg.linear.x = 1.5 + 0.1 * dt
        msg.angular.z = 2.0 / (1.0 + 0.05 * dt)

        self.pub.publish(msg)
        self.get_logger().info(
            f'Moving: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()