"""Publishes the drone's real position for visualization in RViz2.

Converts PX4's NED VehicleLocalPosition into a Z-up PoseStamped, plus an
accumulating Path so RViz2 can draw the actual flown trajectory (capped in
length so it doesn't grow unbounded over a long-running session).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from px4_msgs.msg import VehicleLocalPosition

from px4_mpc_controller.offboard_control import px4_qos_profile
from px4_mpc_controller.frames import ned_to_enu

MAX_PATH_LENGTH = 3000


class VehiclePosePublisherNode(Node):

    def __init__(self):
        super().__init__('vehicle_pose_publisher')

        self._pose_pub = self.create_publisher(PoseStamped, '/viz/vehicle_pose', 10)
        self._path_pub = self.create_publisher(Path, '/viz/vehicle_path', 10)
        self._path = Path()
        self._path.header.frame_id = 'map'

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_position_callback, px4_qos_profile())

    def _local_position_callback(self, msg: VehicleLocalPosition):
        enu = ned_to_enu((msg.x, msg.y, msg.z))

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
    node = VehiclePosePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
