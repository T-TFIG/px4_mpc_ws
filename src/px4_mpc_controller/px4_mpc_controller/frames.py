"""NED <-> ENU conversion, for visualization only.

PX4/px4_msgs use NED (North-East-Down). RViz2/ROS tooling conventionally
expects a Z-up frame (ENU: East-North-Up). This only matters for display --
the control loop (mpc_node, mpc_solver) stays entirely in NED to match PX4.
"""
import numpy as np


def ned_to_enu(position_ned) -> np.ndarray:
    north, east, down = position_ned
    return np.array([east, north, -down])
