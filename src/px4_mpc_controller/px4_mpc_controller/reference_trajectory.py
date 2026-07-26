"""Parametric reference trajectory: a horizontal circle, in PX4's NED frame.

Shared by mpc_node (which needs the full horizon-ahead reference for the
solver) and trajectory_generator (which just publishes the current point for
visualization/logging) so there is exactly one definition of "the path."
"""
import numpy as np


class CircleTrajectory:
    def __init__(self, center_ned, radius: float, angular_rate: float):
        self.center = np.asarray(center_ned, dtype=float)  # [north, east, down]
        self.radius = radius
        self.omega = angular_rate

    def position(self, t: float) -> np.ndarray:
        return self.center + np.array([
            self.radius * np.cos(self.omega * t),
            self.radius * np.sin(self.omega * t),
            0.0,
        ])

    def velocity(self, t: float) -> np.ndarray:
        return np.array([
            -self.radius * self.omega * np.sin(self.omega * t),
            self.radius * self.omega * np.cos(self.omega * t),
            0.0,
        ])

    def horizon(self, t0: float, dt: float, steps: int):
        """Position and velocity references for t0, t0+dt, ..., t0+steps*dt.
        Returns two (3, steps+1) arrays, ready for the solver."""
        times = t0 + dt * np.arange(steps + 1)
        positions = np.stack([self.position(t) for t in times], axis=1)
        velocities = np.stack([self.velocity(t) for t in times], axis=1)
        return positions, velocities
