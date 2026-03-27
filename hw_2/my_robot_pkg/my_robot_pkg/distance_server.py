#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from my_robot_pkg.srv import ComputeDistance
 
class DistanceServer(Node):
    def __init__(self):
        super().__init__('dist_server')
        self.srv = self.create_service(ComputeDistance, 'compute_dist', self.handle_req)
        self.get_logger().info("server started")

 
    def handle_req(self, req, resp):
        dx = req.x2 - req.x1
        dy = req.y2 - req.y1
        resp.distance = math.sqrt(dx**2 + dy**2)
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = DistanceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()