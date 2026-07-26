# px4_mpc_ws

A ROS 2 (Jazzy) workspace for learning PX4 and navigating a simulated multicopter with a
**custom Model Predictive Controller (MPC)**, running fully in Gazebo SITL and fully
containerized in Docker.

This is **not** PX4's built-in `mc_pos_control` position controller (which is also, confusingly,
called "MPC" internally) — it's a from-scratch offboard MPC built with CasADi that computes
position/velocity setpoints and sends them to PX4 over its ROS 2 offboard interface.

## Stack

- ROS 2 Jazzy Jalisco
- PX4-Autopilot `v1.17.0` (SITL only, `gz_x500` target)
- Gazebo Harmonic (`gz-sim`)
- Micro XRCE-DDS Agent + `px4_msgs` (PX4 ↔ ROS 2 bridge)
- CasADi + IPOPT for the MPC solver

## Layout

```
src/
  px4_msgs/             # PX4 message definitions (git submodule, pinned to release/1.17)
  px4_mpc_bringup/       # launch files
  px4_mpc_controller/    # the custom MPC controller package
```

## Getting started

```bash
git clone --recurse-submodules <this repo>
cd px4_mpc_ws
docker compose build          # ~30-60 min the first time (builds PX4, the XRCE agent, the ROS2 workspace)
xhost +local:docker           # one-time, allows the container to open Gazebo's window on your display
docker compose run --rm px4_mpc
```

Inside the container, ROS 2 and the workspace overlay are already sourced via `.bashrc`.

## Status

Build/toolchain scaffolding stage. See the staged plan below — each stage is developed and
verified independently.

- [x] Stage -1 — Docker + GitHub scaffold
- [ ] Stage 0 — PX4 SITL + Gazebo sanity check
- [ ] Stage 1 — PX4 ↔ ROS 2 bridge verified (`/fmu/out/...` topics live)
- [ ] Stage 2 — Minimal offboard control demo (arm, hold a fixed setpoint)
- [ ] Stage 3 — Custom MPC navigation (track a reference trajectory)

## Safety

Simulation (SITL) only. Nothing here talks to real flight hardware.
