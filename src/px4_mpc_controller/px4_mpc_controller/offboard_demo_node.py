"""Stage 2: minimal arm/offboard/hold demo.

Streams OffboardControlMode + a fixed-point TrajectorySetpoint at 10 Hz, then
arms and requests Offboard mode once enough setpoints have been sent -- PX4
requires a short setpoint stream before it will accept the mode switch, and
requires the stream to continue uninterrupted afterwards or it auto-exits
Offboard as a failsafe.
"""
import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

from px4_mpc_controller.offboard_control import OffboardArmingMixin, px4_qos_profile


class OffboardDemoNode(Node, OffboardArmingMixin):

    def __init__(self):
        super().__init__('offboard_demo_node')

        qos_profile = px4_qos_profile()
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)


        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._vehicle_status_callback, qos_profile)

        # NED hover target: 5 metres above the takeoff point (z is Down, so negative).
        self.target_position = (0.0, 5.0, -5.0)

        self._nav_state = None
        self._setpoint_counter = 0
        self.create_timer(0.1, self._timer_callback)  # 10 Hz

    def _vehicle_status_callback(self, msg: VehicleStatus):
        if msg.nav_state != self._nav_state:
            self._nav_state = msg.nav_state
            offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
            self.get_logger().info(
                f'nav_state changed -> {msg.nav_state}'
                + (' (OFFBOARD engaged)' if offboard else ''))

    def _timer_callback(self):
        if self._setpoint_counter == 10:
            self._engage_offboard_mode()
            self._arm()

        self._publish_offboard_control_mode_position()
        self._publish_trajectory_setpoint()

        if self._setpoint_counter < 11:
            self._setpoint_counter += 1

    def _publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = list(self.target_position)
        msg.yaw = 0.0
        msg.timestamp = self._now_us()
        self.trajectory_setpoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
