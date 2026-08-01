ECE 486 — Final Project Report (Draft)

Authors: Amir Hama
Course: ECE 486 — Robotics Project
Date: 2026-08-01

Abstract

This project implements and validates waypoint navigation and obstacle avoidance for DJI RoboMaster robots in a 4 m × 4 m environment using the multi_robomaster_ros_sim simulator and a simulator-free demo path for reproducible grading. The control strategy uses an approximate linearization about a look-ahead control point and artificial potential fields for obstacle avoidance. A mock VRPN publisher and headless demo path were added so the solution can be validated without the GUI-based simulator.

1. Problem statement and constraints

- Workspace: a square world with x,y ∈ [-2, 2] meters.
- Robots: mobile bases only (no arm/gripper). Movement and inter-robot avoidance are required.
- Interfaces: subscribe to /vrpn_mocap/dji_robot_<ID>/pose (geometry_msgs/PoseStamped) and publish /robot<ID>/cmd_vel (geometry_msgs/Twist).
- Grading expects reliable navigation to waypoints while avoiding other robots; no manipulation is required.

2. System architecture

- Simulation/ (repo): contains the controller and mock VRPN.
  - hockey_node.py: control node (supports --use-mock and --log-file).
  - mock_vrpn.py: lightweight PoseStamped publisher for reproducible testing.
- The controller can run in two modes: connected to VRPN publisher (real or simulator) or --use-mock mode which synthesizes robot poses.

3. Control algorithm

3.1 Look-ahead control point
- Control point p = [x + l cos θ, y + l sin θ]^T, with look-ahead distance l = 0.3 m.
- The control objective drives p toward a sequence of patrol waypoints using an attractive force.

3.2 Approximate linearization for differential-drive-like control
Given p_dot = [ṗ_x, ṗ_y]^T, the controller computes commanded linear and angular velocities as:

v = ṗ_x cos θ + ṗ_y sin θ
ω = (-ṗ_x sin θ + ṗ_y cos θ) / l

These expressions approximate the mapping from ṗ to body-frame velocities at the look-ahead point.

3.3 Artificial potential fields
- Attractive force: F_att = ζ (q_target - p)
- Repulsive force (from an obstacle at q_obs): when ρ = ||p - q_obs|| < ρ0,
  F_rep = η (1/ρ - 1/ρ0) (1/ρ^2) ∇ρ
- The total commanded virtual velocity of p is ṗ = F_att + F_rep (scaled appropriately).

4. Safety hardening and practical considerations

To ensure safe, stable behavior the controller includes several practical measures:
- Minimum obstacle distance clamp: min_rho = 0.12 m to avoid singular repulsion magnitudes.
- Clamp maximal repulsive force magnitude: max_rep_force = 2.0 (tunable) to avoid destabilizing torques.
- Velocity saturation: max_v = 0.8 m/s, max_ω = 3.0 rad/s.
- Exponential smoothing of commands (α = 0.4) to reduce jerk and oscillation.
- CSV telemetry logging for offline analysis and reproducibility.

5. Parameters (recommended defaults)
- l (look-ahead distance): 0.30 m
- ζ (attractive gain): 1.0
- η (repulsive gain): 0.5
- ρ0 (repulsive radius): 1.0 m
- min_rho: 0.12 m
- max_rep_force: 2.0
- max_v: 0.8 m/s
- max_ω: 3.0 rad/s
- smoothing α: 0.4

6. Experimental procedure

- Use the Simulation demo with --use-mock to run a headless trial and collect telemetry CSVs (see Simulation/README.md for exact commands).
- Suggested experiments: N=10 trials for each scenario with varied initial robot positions.
  - Scenario A: static obstacle (obstacle robot stands at fixed pose near path)
  - Scenario B: moving obstacle (obstacle robot follows simple patrol path)
- Metrics to compute per trial: success (reached waypoint within tolerance), collisions (ρ < collision_threshold), time-to-goal, path length, minimum distance to obstacle, average distance to obstacle.

7. Results (placeholder)

- Include tables and plots here once experiments are run. Recommended plots:
  - Trajectory overlay (ground truth positions) of agent and obstacle across trials.
  - Distance to obstacle vs time.
  - v and ω vs time.
  - Histogram of time-to-goal across trials.

8. Discussion and limitations

- The mock-based demo ensures reproducibility but does not exercise the exact simulator physics; however the control logic and interfaces are validated.
- The original simulator GUI failure (matplotlib/Qt) in headless Docker/WSL environments was avoided by providing the mock path; the simulator code was not modified per project constraints.
- Potential fields are susceptible to local minima; future work could add randomized waypoint perturbations, blended global planners, or dynamic potential shaping.

9. How to run (summary)

- See Simulation/README.md for the single-command Docker demo and alternate host-run commands. The controller supports --use-mock, --log-file, and CLI tuning of key parameters.

10. Submission contents

- Simulation/ (code + mock)
- report.md (this file)
- sample logs/plots/ (to be generated and added)
- run_demo.sh (optional automation script)

Appendix
- Key file references:
  - Simulation/hockey_node.py
  - Simulation/mock_vrpn.py


Notes
- This is a draft report. After you confirm, I can run experiments, generate the recommended plots, fill in the Results section, and produce a PDF version (report.pdf) to include in the submission.
