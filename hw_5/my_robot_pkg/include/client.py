#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from task_5_turtle.srv import SpawnTurtle, KillTurtle


class Client(Node):
    CENTER_X = 5.5
    CENTER_Y = 5.5
    RADIUS = 3.0

    REQUEST_COUNT = 8
    SPAWN_INTERVAL = 0.7
    KILL_INTERVAL = 0.4
    HOLD_TIME = 5.0

    def __init__(self):
        super().__init__('client')

        self.cli_spawn = self.create_client(SpawnTurtle, 'spawn_turtle')
        self.cli_kill = self.create_client(KillTurtle, 'kill_turtle')

        self.cli_spawn.wait_for_service()
        self.cli_kill.wait_for_service()

        self.names = []

        self.send_spawn_requests()
        time.sleep(self.HOLD_TIME)
        self.send_kill_requests()

    def send_spawn_requests(self):
        for index in range(self.REQUEST_COUNT):
            angle = 2.0 * math.pi * index / self.REQUEST_COUNT

            req = SpawnTurtle.Request()
            req.name = f'circle_{index + 1}'
            req.x = self.CENTER_X + self.RADIUS * math.cos(angle)
            req.y = self.CENTER_Y + self.RADIUS * math.sin(angle)

            self.get_logger().info(
                f'spawn request: {req.name} -> x={req.x:.2f}, y={req.y:.2f}'
            )

            future = self.cli_spawn.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if response is not None and response.success:
                self.names.append(req.name)
                self.get_logger().info(
                    f'spawn response: success={response.success}, message={response.message}'
                )
            else:
                self.get_logger().warning(
                    f'spawn response: failed for {req.name}'
                )

            time.sleep(self.SPAWN_INTERVAL)

    def send_kill_requests(self):
        for name in self.names:
            req = KillTurtle.Request()
            req.name = name

            self.get_logger().info(f'kill request: {name}')

            future = self.cli_kill.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if response is not None:
                self.get_logger().info(
                    f'kill response: success={response.success}, message={response.message}'
                )
            else:
                self.get_logger().warning(
                    f'kill response: failed for {name}'
                )

            time.sleep(self.KILL_INTERVAL)


def main(args=None):
    rclpy.init(args=args)
    node = Client()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()