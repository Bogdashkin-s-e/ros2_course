#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
 
class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)
 
    def callback(self, msg):
        self.get_logger().info(f'x={msg.x:.2f} y={msg.y:.2f} theta={msg.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()