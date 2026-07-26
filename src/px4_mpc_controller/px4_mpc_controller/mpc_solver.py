"""CasADi point-mass MPC for offboard position/velocity tracking.

State x = [px, py, pz, vx, vy, vz] in PX4's NED frame.
Input u = [ax, ay, az] -- commanded acceleration.
Dynamics: double integrator, forward Euler, discretized at dt.

The solved acceleration is not sent to PX4 directly -- the *predicted next
state* (position + velocity) is sent as a TrajectorySetpoint, which PX4's own
position controller then tracks (see offboard_control.py / mpc_node.py for
why: this MPC replaces the guidance/trajectory layer, not PX4's inner loop).
"""
import numpy as np
import casadi as ca


class MpcSolver:
    def __init__(self, horizon_steps: int, dt: float,
                 max_xy_vel: float, max_z_vel: float,
                 max_xy_accel: float, max_z_accel: float,
                 weight_position: float, weight_velocity: float, weight_control: float):
        self.n = horizon_steps
        self.dt = dt
        self.nx = 6
        self.nu = 3

        opti = ca.Opti()
        X = opti.variable(self.nx, self.n + 1)
        U = opti.variable(self.nu, self.n)

        x0 = opti.parameter(self.nx)
        p_ref = opti.parameter(3, self.n + 1)
        v_ref = opti.parameter(3, self.n + 1)

        opti.subject_to(X[:, 0] == x0)

        cost = 0
        for k in range(self.n):
            pos_k = X[0:3, k]
            vel_k = X[3:6, k]
            u_k = U[:, k]

            cost += weight_position * ca.sumsqr(pos_k - p_ref[:, k])
            cost += weight_velocity * ca.sumsqr(vel_k - v_ref[:, k])
            cost += weight_control * ca.sumsqr(u_k)

            pos_next = pos_k + vel_k * dt + 0.5 * u_k * dt ** 2
            vel_next = vel_k + u_k * dt
            opti.subject_to(X[0:3, k + 1] == pos_next)
            opti.subject_to(X[3:6, k + 1] == vel_next)

            opti.subject_to(opti.bounded(-max_xy_accel, u_k[0], max_xy_accel))
            opti.subject_to(opti.bounded(-max_xy_accel, u_k[1], max_xy_accel))
            opti.subject_to(opti.bounded(-max_z_accel, u_k[2], max_z_accel))
            opti.subject_to(opti.bounded(-max_xy_vel, vel_k[0], max_xy_vel))
            opti.subject_to(opti.bounded(-max_xy_vel, vel_k[1], max_xy_vel))
            opti.subject_to(opti.bounded(-max_z_vel, vel_k[2], max_z_vel))

        # terminal cost on the final tracking error
        cost += weight_position * ca.sumsqr(X[0:3, self.n] - p_ref[:, self.n])

        opti.minimize(cost)
        opti.solver('ipopt', {
            'print_time': 0,
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'ipopt.max_iter': 100,
        })

        self._opti = opti
        self._X = X
        self._U = U
        self._x0_param = x0
        self._p_ref_param = p_ref
        self._v_ref_param = v_ref
        self._prev_sol = None

    def solve(self, current_state: np.ndarray, p_ref: np.ndarray, v_ref: np.ndarray):
        """
        current_state: (6,) [px, py, pz, vx, vy, vz]
        p_ref, v_ref: (3, horizon_steps+1) position/velocity references

        Returns (position_setpoint, velocity_setpoint, acceleration_command),
        each a (3,) array -- the predicted state one step ahead, to send to
        PX4 as the next TrajectorySetpoint.
        """
        self._opti.set_value(self._x0_param, current_state)
        self._opti.set_value(self._p_ref_param, p_ref)
        self._opti.set_value(self._v_ref_param, v_ref)

        if self._prev_sol is not None:
            self._opti.set_initial(self._X, self._prev_sol['X'])
            self._opti.set_initial(self._U, self._prev_sol['U'])

        sol = self._opti.solve()

        X_sol = sol.value(self._X)
        U_sol = sol.value(self._U)
        self._prev_sol = {'X': X_sol, 'U': U_sol}

        position_setpoint = X_sol[0:3, 1]
        velocity_setpoint = X_sol[3:6, 1]
        acceleration_command = U_sol[:, 0]
        return position_setpoint, velocity_setpoint, acceleration_command
