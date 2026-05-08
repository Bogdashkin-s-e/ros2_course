#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy


class WallFollower(Node):
    FIND_WALL = 0
    TURN_LEFT = 1
    FOLLOW_WALL = 2

    def __init__(self):
        super().__init__('wall_follower')

        self.SAFE_DIST = 0.5
        self.TURN_DIST = 0.7
        self.WALL_DIST = 0.8
        self.SPEED = 0.2
        self.TURN_SPEED = 0.5

        self.state = self.FIND_WALL
        self.front_dist = float('inf')
        self.right_dist = float('inf')
        self.trajectory = []

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10,
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop,
        )

        self.get_logger().info('Wall Follower Node Started')

    def safe_min(self, ranges):
        valid = [
            d for d in ranges
            if d > 0.1 and d < 3.5
        ]

        return min(valid) if valid else float('inf')

    def scan_callback(self, msg):
        front_ranges = list(msg.ranges[0:15]) + list(msg.ranges[345:360])
        self.front_dist = self.safe_min(front_ranges)

        right_ranges = list(msg.ranges[260:280])
        self.right_dist = self.safe_min(right_ranges)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.trajectory.append((x, y))

        if len(self.trajectory) % 100 == 0:
            self.get_logger().info(
                f'Path length: {len(self.trajectory)} points'
            )

    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if self.state == self.FIND_WALL:
            if self.front_dist < self.SAFE_DIST:
                self.state = self.TURN_LEFT
                self.get_logger().info('FIND_WALL -> TURN_LEFT')

                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.TURN_SPEED
            else:
                cmd.twist.linear.x = self.SPEED
                cmd.twist.angular.z = 0.0

        elif self.state == self.TURN_LEFT:
            if self.front_dist > self.TURN_DIST:
                self.state = self.FOLLOW_WALL
                self.get_logger().info('TURN_LEFT -> FOLLOW_WALL')

                cmd.twist.linear.x = self.SPEED
                cmd.twist.angular.z = 0.0
            else:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.TURN_SPEED

        elif self.state == self.FOLLOW_WALL:
            if self.front_dist < self.SAFE_DIST:
                self.state = self.TURN_LEFT
                self.get_logger().info('FOLLOW_WALL -> TURN_LEFT')

                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.TURN_SPEED

            elif self.right_dist > 1.0:
                self.state = self.FIND_WALL
                self.get_logger().info('FOLLOW_WALL -> FIND_WALL')

                cmd.twist.linear.x = self.SPEED
                cmd.twist.angular.z = 0.0

            else:
                error = self.WALL_DIST - self.right_dist

                cmd.twist.linear.x = self.SPEED
                cmd.twist.angular.z = error * 0.5

        self.cmd_pub.publish(cmd)

    def save_trajectory(
        self,
        filename='/root/ros2_course/TASK_7/trajectory.txt',
    ):
        with open(filename, 'w') as f:
            for x, y in self.trajectory:
                f.write(f'{x} {y}\n')

        self.get_logger().info(f'Trajectory saved to {filename}')


def main(args=None):
    rclpy.init(args=args)

    node = WallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        node.save_trajectory()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
