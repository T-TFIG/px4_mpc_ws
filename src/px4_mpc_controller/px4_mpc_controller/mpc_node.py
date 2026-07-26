"""Stage 3: MPC-driven offboard navigation node. Not yet implemented."""
import rclpy
from rclpy.node import Node


class MpcNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        self.get_logger().warn('mpc_node is a Stage 3 placeholder — MPC control loop not yet implemented')


def main(args=None):
    rclpy.init(args=args)
    node = MpcNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
