"""Stage 2: minimal arm/offboard/hold demo. Not yet implemented."""
import rclpy
from rclpy.node import Node


class OffboardDemoNode(Node):
    def __init__(self):
        super().__init__('offboard_demo_node')
        self.get_logger().warn(
            'offboard_demo_node is a Stage 2 placeholder — arm/offboard/hold logic not yet implemented')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
