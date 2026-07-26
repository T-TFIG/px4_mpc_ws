"""Shared PX4 offboard-mode helpers.

The arm + mode-switch handshake and the QoS profile PX4's uXRCE-DDS bridge
requires are identical for every node that drives PX4 over /fmu/in/* -- kept
here once instead of duplicated in offboard_demo_node and mpc_node.
"""
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleCommand

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
# Bypasses preflight/health checks (same as `commander arm -f`) -- needed since
# we have no GCS/RC connected; see NAV_DLL_ACT in sitl/4001_gz_x500.post.
PX4_ARM_DISARM_FORCE_MAGIC = 21196.0


def px4_qos_profile() -> QoSProfile:
    """PX4's uXRCE-DDS bridge is best-effort/VOLATILE (ROS2's "sensor_data" preset).
    TRANSIENT_LOCAL durability is DDS-incompatible with it and silently drops
    every message -- no error, messages just never match."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class OffboardArmingMixin:
    """Mix into an rclpy.Node that has self.vehicle_command_pub and
    self.offboard_control_mode_pub (both created with px4_qos_profile())."""

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_offboard_control_mode_position(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = self._now_us()
        self.offboard_control_mode_pub.publish(msg)

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.vehicle_command_pub.publish(msg)

    def _engage_offboard_mode(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD))
        self.get_logger().info('Requested Offboard mode')

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0, param2=PX4_ARM_DISARM_FORCE_MAGIC)
        self.get_logger().info('Requested arm (forced)')
