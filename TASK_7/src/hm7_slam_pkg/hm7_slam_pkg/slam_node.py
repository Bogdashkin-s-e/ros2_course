#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


GridCell = Tuple[int, int]


def quaternion_to_yaw(x, y, z, w):
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(sin_yaw, cos_yaw)


class SlamNode(Node):
    def __init__(self):
        super().__init__('custom_slam_node')

        self.resolution = 0.05
        self.map_size = 16.0

        self.width = int(self.map_size / self.resolution)
        self.height = int(self.map_size / self.resolution)

        self.origin_x = -self.map_size / 2.0
        self.origin_y = -self.map_size / 2.0

        self.log_odds = [0.0 for _ in range(self.width * self.height)]
        self.observed = [False for _ in range(self.width * self.height)]

        self.l_occ = 0.85
        self.l_free = -0.40
        self.l_min = -5.0
        self.l_max = 5.0

        self.max_lidar_range = 3.5
        self.scan_step = 2

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos,
        )

        self.timer = self.create_timer(0.5, self.publish_map)

        self.get_logger().info('Custom SLAM node started')
        self.get_logger().info(
            f'Map size: {self.width} x {self.height}, resolution: {self.resolution}'
        )

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.robot_x = position.x
        self.robot_y = position.y
        self.robot_yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        self.has_odom = True

    def scan_callback(self, msg):
        if not self.has_odom:
            return

        robot_cell = self.world_to_map(self.robot_x, self.robot_y)

        if robot_cell is None:
            self.get_logger().warn(
                'Robot is outside map bounds',
                throttle_duration_sec=2.0,
            )
            return

        usable_max_range = min(msg.range_max, self.max_lidar_range)

        for i in range(0, len(msg.ranges), self.scan_step):
            distance = msg.ranges[i]

            if math.isnan(distance) or distance <= msg.range_min:
                continue

            has_hit = True

            if math.isinf(distance) or distance >= usable_max_range:
                distance = usable_max_range
                has_hit = False

            angle = msg.angle_min + i * msg.angle_increment
            global_angle = self.robot_yaw + angle

            hit_x = self.robot_x + distance * math.cos(global_angle)
            hit_y = self.robot_y + distance * math.sin(global_angle)

            end_cell = self.world_to_map_unbounded(hit_x, hit_y)

            self.update_ray(robot_cell, end_cell, has_hit)

    def update_ray(self, start_cell, end_cell, has_hit):
        cells = self.bresenham(
            start_cell[0],
            start_cell[1],
            end_cell[0],
            end_cell[1],
        )

        if len(cells) == 0:
            return

        for cell in cells[:-1]:
            x, y = cell

            if not self.is_inside_map(x, y):
                return

            self.update_cell(x, y, self.l_free)

        if has_hit:
            x, y = cells[-1]

            if self.is_inside_map(x, y):
                self.update_cell(x, y, self.l_occ)

    def update_cell(self, x, y, delta):
        index = self.grid_index(x, y)

        new_value = self.log_odds[index] + delta
        new_value = max(self.l_min, min(self.l_max, new_value))

        self.log_odds[index] = new_value
        self.observed[index] = True

    def publish_map(self):
        msg = OccupancyGrid()

        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.header.frame_id = 'odom'

        msg.info.map_load_time = now
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.make_occupancy_data()

        self.map_pub.publish(msg)

    def make_occupancy_data(self):
        data = []

        for is_observed, value in zip(self.observed, self.log_odds):
            if not is_observed:
                data.append(-1)
                continue

            probability = self.log_odds_to_probability(value)

            if probability < 0.35:
                data.append(0)
            elif probability > 0.65:
                data.append(100)
            else:
                data.append(int(probability * 100))

        return data

    def log_odds_to_probability(self, value):
        return 1.0 - 1.0 / (1.0 + math.exp(value))

    def world_to_map(self, x, y) -> Optional[GridCell]:
        grid_x, grid_y = self.world_to_map_unbounded(x, y)

        if not self.is_inside_map(grid_x, grid_y):
            return None

        return grid_x, grid_y

    def world_to_map_unbounded(self, x, y) -> GridCell:
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)

        return grid_x, grid_y

    def is_inside_map(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def grid_index(self, x, y):
        return y * self.width + x

    def bresenham(self, x0, y0, x1, y1) -> List[GridCell]:
        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        error = dx - dy

        x = x0
        y = y0

        while True:
            cells.append((x, y))

            if x == x1 and y == y1:
                break

            error2 = 2 * error

            if error2 > -dy:
                error -= dy
                x += sx

            if error2 < dx:
                error += dx
                y += sy

        return cells


def main(args=None):
    rclpy.init(args=args)

    node = SlamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
