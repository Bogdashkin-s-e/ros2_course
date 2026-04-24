#!/usr/bin/env python3

import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def filtered_min(values, default=3.5):
    valid = [
        value for value in values
        if math.isfinite(value) and 0.12 < value < 3.5
    ]

    if not valid:
        return default

    valid.sort()

    # Берём не один минимальный луч, а среднее нескольких ближайших.
    # Так меньше дёрганий от шума лидара.
    sample = valid[:min(3, len(valid))]
    return sum(sample) / len(sample)


def smooth(previous, current, alpha=0.65):
    if not math.isfinite(previous):
        return current
    return alpha * previous + (1.0 - alpha) * current


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WallFollower(Node):
    FIND_WALL = 0
    TURN_LEFT = 1
    FOLLOW_WALL = 2

    def __init__(self):
        super().__init__('wall_follower')

        self.state = self.FIND_WALL
        self.state_enter_time_ns = self.get_clock().now().nanoseconds

        self.front_dist = float('inf')
        self.front_right_dist = float('inf')
        self.right_dist = float('inf')
        self.has_scan = False

        # Дистанции
        self.front_blocked_dist = 0.48
        self.front_clear_dist = 0.68

        self.desired_right_dist = 0.55
        self.right_found_dist = 0.85
        self.right_lost_dist = 1.15

        self.front_right_blocked_dist = 0.38

        # Скорости
        self.follow_linear_speed = 0.18
        self.search_linear_speed = 0.15

        self.search_turn_speed = -0.35
        self.turn_left_speed = 0.75

        # P-регулятор движения вдоль правой стены
        self.follow_gain = 1.7
        self.front_right_gain = 0.65
        self.max_angular_speed = 0.85

        # Чтобы состояние TURN_LEFT не переключалось туда-сюда слишком быстро
        self.min_turn_time_sec = 0.35

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/wall_follower_path', 10)

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.last_saved_x = None
        self.last_saved_y = None

        self.csv_path = os.path.expanduser(
            '~/TASK_6/wall_follower_path.csv'
        )
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time_sec', 'x', 'y', 'yaw'])

        self.get_logger().info('Wall Follower Node Started')
        self.get_logger().info(f'Path will be saved to {self.csv_path}')

    def state_name(self, state):
        if state == self.FIND_WALL:
            return 'find_wall'
        if state == self.TURN_LEFT:
            return 'turn_left'
        return 'follow_wall'

    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(
                f'State: {self.state_name(self.state)} -> {self.state_name(new_state)}'
            )
            self.state = new_state
            self.state_enter_time_ns = self.get_clock().now().nanoseconds

    def state_elapsed_sec(self):
        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self.state_enter_time_ns) / 1e9

    def scan_callback(self, msg):
        ranges = list(msg.ranges)

        # TurtleBot3 LDS: индекс примерно соответствует градусу.
        # 0 градусов — впереди, 90 — слева, 270 — справа.
        raw_front = filtered_min(
            ranges[0:18] + ranges[342:360],
            default=3.5,
        )

        raw_front_right = filtered_min(
            ranges[300:340],
            default=3.5,
        )

        raw_right = filtered_min(
            ranges[255:285],
            default=3.5,
        )

        self.front_dist = smooth(self.front_dist, raw_front)
        self.front_right_dist = smooth(self.front_right_dist, raw_front_right)
        self.right_dist = smooth(self.right_dist, raw_right)

        self.has_scan = True

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        need_save = False
        if self.last_saved_x is None or self.last_saved_y is None:
            need_save = True
        else:
            distance = math.hypot(
                position.x - self.last_saved_x,
                position.y - self.last_saved_y,
            )
            if distance >= 0.05:
                need_save = True

        if not need_save:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv_writer.writerow([
            f'{time_sec:.3f}',
            f'{position.x:.4f}',
            f'{position.y:.4f}',
            f'{yaw:.4f}',
        ])
        self.csv_file.flush()

        self.last_saved_x = position.x
        self.last_saved_y = position.y

    def make_cmd(self, linear_x=0.0, angular_z=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z
        return cmd

    def follow_wall_cmd(self):
        # Если right_dist больше желаемого — робот далеко от стены,
        # значит надо довернуть направо: angular отрицательный.
        # Если right_dist меньше желаемого — робот близко к стене,
        # значит надо отвернуть налево: angular положительный.
        wall_error = self.desired_right_dist - self.right_dist
        angular = self.follow_gain * wall_error

        # Если впереди-справа стена близко, заранее отворачиваем налево.
        if self.front_right_dist < self.front_right_blocked_dist:
            angular += self.front_right_gain * (
                self.front_right_blocked_dist - self.front_right_dist
            )

        angular = clamp(
            angular,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        # На сильных поворотах едем чуть медленнее.
        turn_ratio = abs(angular) / self.max_angular_speed
        linear = self.follow_linear_speed * (1.0 - 0.35 * turn_ratio)
        linear = max(0.10, linear)

        return self.make_cmd(linear, angular)

    def control_loop(self):
        if not self.has_scan:
            return

        cmd = self.make_cmd(0.0, 0.0)

        if self.state == self.FIND_WALL:
            if self.front_dist < self.front_blocked_dist:
                self.set_state(self.TURN_LEFT)
                cmd = self.make_cmd(0.0, self.turn_left_speed)

            elif self.right_dist < self.right_found_dist:
                self.set_state(self.FOLLOW_WALL)
                cmd = self.follow_wall_cmd()

            else:
                # Ищем правую стену: едем вперед и плавно заворачиваем направо.
                cmd = self.make_cmd(
                    self.search_linear_speed,
                    self.search_turn_speed,
                )

        elif self.state == self.TURN_LEFT:
            can_leave_turn = (
                self.state_elapsed_sec() >= self.min_turn_time_sec
                and self.front_dist > self.front_clear_dist
                and self.front_right_dist > self.front_right_blocked_dist
            )

            if can_leave_turn:
                if self.right_dist < self.right_lost_dist:
                    self.set_state(self.FOLLOW_WALL)
                    cmd = self.follow_wall_cmd()
                else:
                    self.set_state(self.FIND_WALL)
                    cmd = self.make_cmd(
                        self.search_linear_speed,
                        self.search_turn_speed,
                    )
            else:
                cmd = self.make_cmd(0.0, self.turn_left_speed)

        elif self.state == self.FOLLOW_WALL:
            if (
                self.front_dist < self.front_blocked_dist
                or self.front_right_dist < self.front_right_blocked_dist
            ):
                self.set_state(self.TURN_LEFT)
                cmd = self.make_cmd(0.0, self.turn_left_speed)

            elif (
                self.right_dist > self.right_lost_dist
                and self.front_right_dist > self.right_lost_dist
            ):
                self.set_state(self.FIND_WALL)
                cmd = self.make_cmd(
                    self.search_linear_speed,
                    self.search_turn_speed,
                )

            else:
                cmd = self.follow_wall_cmd()

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(self.make_cmd(0.0, 0.0))

    def close_file(self):
        if not self.csv_file.closed:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.stop_robot()
        node.close_file()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()