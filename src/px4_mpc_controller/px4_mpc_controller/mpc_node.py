"""Stage 3: MPC-driven offboard navigation.

Same arm/offboard handshake as offboard_demo_node (see offboard_control.py),
but instead of holding a fixed setpoint, re-solves the MPC every tick against
an analytic reference trajectory and sends the solver's predicted next
position/velocity/acceleration as the TrajectorySetpoint.
"""
import numpy as np
import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from px4_mpc_controller.offboard_control import OffboardArmingMixin, px4_qos_profile
from px4_mpc_controller.reference_trajectory import CircleTrajectory
from px4_mpc_controller.mpc_solver import MpcSolver


class MpcNode(Node, OffboardArmingMixin):

    def __init__(self):
        super().__init__('mpc_node')
        self._declare_parameters()
        p = self._read_parameters()

        qos_profile = px4_qos_profile()
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_position_callback, qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._vehicle_status_callback, qos_profile)

        self._trajectory = CircleTrajectory(
            p['trajectory_center'], p['trajectory_radius'], p['trajectory_angular_rate'])
        self._solver = MpcSolver(
            horizon_steps=p['horizon_steps'], dt=p['dt'],
            max_xy_vel=p['max_xy_vel'], max_z_vel=p['max_z_vel'],
            max_xy_accel=p['max_xy_accel'], max_z_accel=p['max_z_accel'],
            weight_position=p['weight_position'], weight_velocity=p['weight_velocity'],
            weight_control=p['weight_control'])
        self._dt = p['dt']

        self._current_state = None  # (6,) [px,py,pz,vx,vy,vz], set once feedback arrives
        self._nav_state = None
        self._setpoint_counter = 0
        self._t0 = None  # sim-relative start time for the trajectory, set on first tick

        # Arming state machine (see _try_arm)
        self._armed = False
        self._preflight_ok = False
        self._last_arm_attempt = -1e9
        self._last_wait_log = -1e9
        self._arm_retry_period = 2.0  # seconds between arm attempts

        self.create_timer(self._dt, self._timer_callback)

    def _declare_parameters(self):
        for name, default in [
            ('horizon_steps', 20), ('dt', 0.1),
            ('max_xy_vel', 5.0), ('max_z_vel', 2.0),
            ('max_xy_accel', 3.0), ('max_z_accel', 2.0),
            ('weight_position', 10.0), ('weight_velocity', 1.0), ('weight_control', 0.1),
            ('trajectory_center', [0.0, 0.0, -5.0]),
            ('trajectory_radius', 5.0), ('trajectory_angular_rate', 0.314159),
        ]:
            self.declare_parameter(name, default)

    def _read_parameters(self) -> dict:
        names = ['horizon_steps', 'dt', 'max_xy_vel', 'max_z_vel', 'max_xy_accel', 'max_z_accel',
                  'weight_position', 'weight_velocity', 'weight_control',
                  'trajectory_center', 'trajectory_radius', 'trajectory_angular_rate']
        return {name: self.get_parameter(name).value for name in names}

    def _local_position_callback(self, msg: VehicleLocalPosition):
        self._current_state = np.array(
            [msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz], dtype=float)

    def _vehicle_status_callback(self, msg: VehicleStatus):
        self._armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self._preflight_ok = msg.pre_flight_checks_pass

        if msg.nav_state != self._nav_state:
            self._nav_state = msg.nav_state
            offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
            self.get_logger().info(
                f'nav_state changed -> {msg.nav_state}'
                + (' (OFFBOARD engaged)' if offboard else ''))

    def _timer_callback(self):
        self._publish_offboard_control_mode_position()

        if self._current_state is None:
            # No feedback yet -- keep streaming the OffboardControlMode heartbeat, but
            # don't start the arm countdown until we have a real setpoint to send: PX4
            # needs an actual TrajectorySetpoint stream (not just the heartbeat) before
            # it will accept the mode switch, or it silently falls back (e.g. to
            # AUTO_LOITER) instead of engaging Offboard.
            return

        if self._t0 is None:
            self._t0 = self._elapsed_seconds()

        t = self._elapsed_seconds() - self._t0
        p_ref, v_ref = self._trajectory.horizon(t, self._dt, self._solver.n)

        position_sp, velocity_sp, accel_sp = self._solver.solve(self._current_state, p_ref, v_ref)
        self._publish_trajectory_setpoint(position_sp, velocity_sp, accel_sp)

        self._setpoint_counter += 1
        self._try_arm()

    def _try_arm(self):
        """Request offboard + arm, retrying until PX4 confirms it is armed.

        This must be a retry loop, not a one-shot. PX4 rejects arming
        (VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED) until its own health checks pass --
        the EKF heading estimate in particular can take tens of seconds to settle after
        boot, and the battery estimate is briefly invalid right after startup. Firing
        the arm command once on a fixed timer means that if either is momentarily
        unhappy, the vehicle stays disarmed forever while the node happily streams
        setpoints, which looks like a controller bug but is not one.

        We also wait for pre_flight_checks_pass before even asking, so we're not
        spamming commands PX4 is guaranteed to refuse.
        """
        if self._armed:
            return

        # Stream setpoints briefly before requesting the mode switch; PX4 requires an
        # established setpoint stream before it will accept Offboard.
        if self._setpoint_counter < 10:
            return

        if not self._preflight_ok:
            now = self._elapsed_seconds()
            if now - self._last_wait_log > 5.0:
                self._last_wait_log = now
                self.get_logger().info(
                    'waiting for PX4 preflight checks to pass before arming '
                    '(EKF heading and battery estimates settle a few seconds after boot)')
            return

        now = self._elapsed_seconds()
        if now - self._last_arm_attempt < self._arm_retry_period:
            return
        self._last_arm_attempt = now

        self._engage_offboard_mode()
        self._arm()

    def _elapsed_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_trajectory_setpoint(self, position, velocity, acceleration):
        msg = TrajectorySetpoint()
        msg.position = [float(v) for v in position]
        msg.velocity = [float(v) for v in velocity]
        msg.acceleration = [float(v) for v in acceleration]
        msg.yaw = 0.0
        msg.timestamp = self._now_us()
        self.trajectory_setpoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MpcNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
