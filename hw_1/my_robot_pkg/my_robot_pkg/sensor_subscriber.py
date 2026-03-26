#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_pkg.msg import SensorData
 
class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_sub')
        self.sub = self.create_subscription(SensorData, 'sensor_topic', self.callback, 10)
 
    def callback(self, msg):
        self.get_logger().info(f'{msg.sensor_name}: {msg.temperature:.1f}C')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()