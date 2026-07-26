"""Stage 3: reference trajectory publisher (circle/figure-8). Not yet implemented."""
import rclpy
from rclpy.node import Node


class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.get_logger().warn(
            'trajectory_generator is a Stage 3 placeholder — reference trajectory publishing not yet implemented')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
