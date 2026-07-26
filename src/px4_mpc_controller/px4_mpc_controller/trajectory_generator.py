"""Publishes the reference trajectory for visualization only.

mpc_node does NOT depend on this running -- it computes the same analytic
trajectory itself (see reference_trajectory.py) to build its horizon. This
node exists purely so you can watch the reference path in RViz2 (or
rqt_plot/echo it) alongside the drone's actual position
(vehicle_pose_publisher.py) to see tracking performance.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from px4_mpc_controller.reference_trajectory import CircleTrajectory
from px4_mpc_controller.frames import ned_to_enu

MAX_PATH_LENGTH = 3000


class TrajectoryGeneratorNode(Node):

    def __init__(self):
        super().__init__('trajectory_generator')
        self.declare_parameter('trajectory_center', [0.0, 0.0, -5.0])
        self.declare_parameter('trajectory_radius', 5.0)
        self.declare_parameter('trajectory_angular_rate', 0.314159)
        self.declare_parameter('dt', 0.1)

        self._trajectory = CircleTrajectory(
            self.get_parameter('trajectory_center').value,
            self.get_parameter('trajectory_radius').value,
            self.get_parameter('trajectory_angular_rate').value)

        self._pose_pub = self.create_publisher(PoseStamped, '/viz/reference_pose', 10)
        self._path_pub = self.create_publisher(Path, '/viz/reference_path', 10)
        self._path = Path()
        self._path.header.frame_id = 'map'

        self._t0 = self.get_clock().now()
        dt = self.get_parameter('dt').value
        self.create_timer(dt, self._timer_callback)

    def _timer_callback(self):
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        enu = ned_to_enu(self._trajectory.position(t))

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(enu[0])
        pose.pose.position.y = float(enu[1])
        pose.pose.position.z = float(enu[2])
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        self._path.header.stamp = pose.header.stamp
        self._path.poses.append(pose)
        if len(self._path.poses) > MAX_PATH_LENGTH:
            self._path.poses.pop(0)
        self._path_pub.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
