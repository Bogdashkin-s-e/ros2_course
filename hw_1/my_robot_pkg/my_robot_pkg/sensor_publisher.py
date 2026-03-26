#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from my_robot_pkg.msg import SensorData
 
class SensorPublisher(Node):
    PUBLISH_HZ = 2.0

    def __init__(self):
        super().__init__('sensor_pub')
        self.pub = self.create_publisher(SensorData, 'sensor_topic', 10)
        period = 1.0 / self.PUBLISH_HZ
        self.timer = self.create_timer(period, self.cb)
 
    def cb(self):
        msg = SensorData()
        msg.sensor_name = 'temp_01'
        msg.temperature = random.uniform(15.0, 45.0)
        self.get_logger().info(f' published: {msg.sensor_name} -> {msg.temperature:.1f}C')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()