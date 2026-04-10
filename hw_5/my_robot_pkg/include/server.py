#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, SetPen
from task_5_turtle.srv import SpawnTurtle, KillTurtle


class Server(Node):
    COLORS = [
        (255, 0, 0),
        (0, 255, 0),
        (0, 0, 255),
        (255, 255, 0),
        (255, 0, 255),
        (0, 255, 255),
        (255, 128, 0),
        (128, 0, 255),
    ]

    def __init__(self):
        super().__init__('server')

        self.srv_spawn = self.create_service(
            SpawnTurtle, 'spawn_turtle', self.handle_spawn_request
        )
        self.srv_kill = self.create_service(
            KillTurtle, 'kill_turtle', self.handle_kill_request
        )

        self.cli_spawn = self.create_client(Spawn, '/spawn')
        self.cli_kill = self.create_client(Kill, '/kill')

        self.cli_spawn.wait_for_service()
        self.cli_kill.wait_for_service()

        self.pen_clients = {}
        self.color_index = 0

        self.get_logger().info('server started')

    def handle_spawn_request(self, req, resp):
        if not (1.0 <= req.x <= 10.0 and 1.0 <= req.y <= 10.0):
            resp.success = False
            resp.message = 'Coords out of safe range'
            return resp

        color = self.COLORS[self.color_index % len(self.COLORS)]
        self.color_index += 1

        spawn_req = Spawn.Request()
        spawn_req.x = req.x
        spawn_req.y = req.y
        spawn_req.theta = 0.0
        spawn_req.name = req.name

        future = self.cli_spawn.call_async(spawn_req)
        future.add_done_callback(
            lambda fut, turtle_name=req.name, x=req.x, y=req.y, color=color:
            self.on_spawn_done(fut, turtle_name, x, y, color)
        )

        resp.success = True
        resp.message = f'Spawn request accepted for {req.name}'
        return resp

    def on_spawn_done(self, future, turtle_name, x, y, color):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to spawn {turtle_name}: {exc}')
            return

        spawned_name = result.name if result is not None and result.name else turtle_name
        self.get_logger().info(
            f'Spawned {spawned_name} at ({x:.2f}, {y:.2f}) with RGB{color}'
        )
        self.set_turtle_pen(spawned_name, color)

    def handle_kill_request(self, req, resp):
        kill_req = Kill.Request()
        kill_req.name = req.name

        future = self.cli_kill.call_async(kill_req)
        future.add_done_callback(
            lambda fut, turtle_name=req.name: self.on_kill_done(fut, turtle_name)
        )

        resp.success = True
        resp.message = f'Kill request accepted for {req.name}'
        return resp

    def on_kill_done(self, future, turtle_name):
        try:
            future.result()
            self.get_logger().info(f'Killed {turtle_name}')
        except Exception as exc:
            self.get_logger().error(f'Failed to kill {turtle_name}: {exc}')

    def set_turtle_pen(self, turtle_name, color):
        if turtle_name not in self.pen_clients:
            self.pen_clients[turtle_name] = self.create_client(
                SetPen, f'/{turtle_name}/set_pen'
            )

        pen_client = self.pen_clients[turtle_name]

        if not pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                f'SetPen service is unavailable for {turtle_name}'
            )
            return

        pen_req = SetPen.Request()
        pen_req.r = color[0]
        pen_req.g = color[1]
        pen_req.b = color[2]
        pen_req.width = 3
        pen_req.off = 0

        pen_client.call_async(pen_req)


def main(args=None):
    rclpy.init(args=args)
    node = Server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()