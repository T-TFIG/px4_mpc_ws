# px4_mpc_ws

A ROS 2 (Jazzy) workspace that navigates a simulated multicopter with a **custom Model
Predictive Controller**, running in Gazebo SITL against real PX4 firmware, fully containerized
in Docker.

This is **not** PX4's built-in `mc_pos_control` position controller (which is also, confusingly,
prefixed `MPC_` internally). It is a from-scratch optimization-based controller built with
CasADi: it solves a constrained optimal control problem at 10 Hz and streams the result to PX4
as offboard setpoints.

## What it does

The controller tracks a reference trajectory (a circle, by default) by solving this problem
every control tick, applying only the first step, and re-solving from the new measurement:

- **Model** — double integrator, discretized exactly under zero-order hold
- **Cost** — quadratic penalty on position error, velocity error, and control effort
- **Constraints** — velocity and acceleration bounds matching PX4's own limits
- **Solver** — CasADi + IPOPT, multiple shooting, warm-started from the previous solution

The result is a **convex QP** with a unique global optimum and predictable solve time.

## Stack

- ROS 2 Jazzy Jalisco
- PX4-Autopilot `v1.17.0` (SITL, `gz_x500` airframe)
- Gazebo Harmonic (`gz-sim`)
- Micro XRCE-DDS Agent + `px4_msgs` (PX4 ↔ ROS 2 bridge)
- CasADi + IPOPT

## Quick start

```bash
git clone --recurse-submodules https://github.com/T-TFIG/px4_mpc_ws.git
cd px4_mpc_ws
docker compose build      # 30-60 min: builds PX4 from source, XRCE agent, ROS 2 workspace
xhost +local:docker       # once per login session, lets the container open Gazebo/RViz
docker compose up -d
```

Then, in two container terminals (`docker compose exec -it px4_mpc bash`):

```bash
# Terminal 1 — PX4 SITL + Gazebo
cd /PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2 — the MPC stack
ros2 launch px4_mpc_bringup mpc_nav.launch.py
```

The drone arms itself, climbs to ~5 m, and flies a 5 m circle.

**Full instructions, verification steps, tuning, and troubleshooting:
[`docs/RUNNING.md`](docs/RUNNING.md).**

## Layout

```
Dockerfile                       # PX4 + Gazebo + XRCE agent + ROS 2 workspace
docker-compose.yaml              # host networking, X11 + GPU passthrough, ./src bind mount
sitl/4001_gz_x500.post           # SITL-only PX4 param overrides, auto-applied at boot
docs/RUNNING.md                  # how to build, run, verify, tune, and debug
src/
  px4_msgs/                      # PX4 message definitions (submodule, release/1.17)
  px4_mpc_bringup/               # launch files
  px4_mpc_controller/
    mpc_solver.py                # the MPC: model, cost, constraints, solve
    mpc_node.py                  # ROS 2 node wrapping the solver
    reference_trajectory.py      # analytic circle reference
    offboard_control.py          # shared PX4 QoS + arm/offboard handshake
    frames.py                    # NED <-> ENU, for visualization only
    docs/                        # derivations (see below)
```

## Documentation

| Document | Contents |
|---|---|
| [`docs/RUNNING.md`](docs/RUNNING.md) | Build, run, verify, tune, troubleshoot |
| [`MPC_explanation.md`](src/px4_mpc_controller/docs/MPC_explanation.md) | Full Euler-Lagrange derivation of the quadrotor dynamics, then MPC from first principles: prediction horizon, receding horizon, cost and constraint design, Lagrange multipliers, KKT conditions |
| [`MPC_solver.md`](src/px4_mpc_controller/docs/MPC_solver.md) | How the optimization is actually solved: RK4, multiple shooting, Newton-type methods, SQP vs interior-point, automatic differentiation, real-time iteration |

Thai translations of both derivation documents are available as `*_th.md`.

## Status

| Stage | State |
|---|---|
| Docker + GitHub scaffold | done |
| PX4 SITL + Gazebo sanity check | done |
| PX4 ↔ ROS 2 bridge (`/fmu/*` topics live) | done |
| Minimal offboard demo (arm, hold a setpoint) | done |
| Point-mass MPC trajectory tracking | done |
| RViz2 visualization of actual vs reference path | done |

### Known limitations

Documented honestly rather than hidden, with fuller discussion in `MPC_solver.md` Part 11:

- **An NLP solver on a QP.** IPOPT is a general nonlinear solver; this problem is a convex QP.
  A dedicated QP solver (OSQP, qpOASES) would very likely be faster with no change to the
  formulation.
- **Hard constraints can go infeasible.** If measured velocity ever exceeds `max_xy_vel`, the
  initial-condition and bound constraints contradict, the solve fails, and the node dies. Soft
  constraints with slack variables are the standard fix; not yet implemented.
- **No state estimator.** Raw `VehicleLocalPosition` is fed straight to the solver, so the
  controller reacts to measurement noise as though it were real motion.
- **No principled terminal cost.** The terminal cost reuses the stage position cost rather than
  an LQR cost-to-go, so there is no closed-loop stability guarantee.

### Not implemented

The full nonlinear (12-state rigid-body) MPC is **derived and documented** in
`MPC_explanation.md` and `MPC_solver.md`, but deliberately not implemented. It is a genuinely
larger undertaking — a nonconvex NLP requiring a real-time-capable solver such as `acados`, and
a different PX4 interface (attitude/body-rate setpoints instead of `TrajectorySetpoint`). The
reasoning behind stopping at the point-mass model is recorded in `MPC_explanation.md`.

## Safety

Simulation only. Nothing here talks to real flight hardware. The PX4 parameter overrides in
`sitl/` disable safety checks that exist for good reasons on a real aircraft — see
`docs/RUNNING.md` Section 5 before considering hardware.
