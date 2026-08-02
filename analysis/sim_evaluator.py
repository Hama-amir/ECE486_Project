#!/usr/bin/env python3
"""
sim_evaluator.py
A lightweight headless simulator that reproduces the controller behavior (look-ahead + potential fields)
and simulates robot kinematics to generate CSV logs similar to the ROS controller logs. This allows evaluation
without Docker/ROS.
"""
import argparse
import os
import math
import random

parser = argparse.ArgumentParser()
parser.add_argument('--n', type=int, default=10, help='Number of trials')
parser.add_argument('--duration', type=float, default=12.0, help='Duration per trial (s)')
parser.add_argument('--dt', type=float, default=0.05, help='Simulation timestep (s)')
parser.add_argument('--outdir', type=str, default=os.path.join(os.getcwd(), 'results'), help='Output directory for CSVs')
parser.add_argument('--seed', type=int, default=0, help='Random seed')
args = parser.parse_args()

random.seed(args.seed)
os.makedirs(args.outdir, exist_ok=True)

# Controller-driven evaluator: import the real compute function from hockey_node
import os
import sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
try:
    # import the module as a plain python module - this must not instantiate rclpy node
    from Simulation.hockey_node import compute_control_from_state, _DEFAULTS
except Exception:
    try:
        from hockey_node import compute_control_from_state, _DEFAULTS
    except Exception as e:
        raise

# waypoint target (single goal for tests)
target_x = 1.5
target_y = 0.0

for ti in range(args.n):
    # randomize initial pose in [-1.5,1.5]
    x = float(random.uniform(-1.5, 1.5))
    y = float(random.uniform(-1.5, 1.5))
    theta = float(random.uniform(-math.pi, math.pi))

    # obstacle behavior: a moving obstacle on a small circle (center near origin)
    obs_center_x = 0.0
    obs_center_y = 0.0
    obs_radius = 0.6
    obs_omega = 0.6  # rad/s

    t = 0.0
    steps = int(args.duration / args.dt)

    # smoothing states
    v_s = 0.0
    w_s = 0.0

    rows = []
    for k in range(steps):
        # obstacle pose
        obs_x = obs_center_x + obs_radius * math.cos(obs_omega * t + ti * 0.4)
        obs_y = obs_center_y + obs_radius * math.sin(obs_omega * t + ti * 0.4)

        # control point p
        p_x = x + _DEFAULTS['l'] * math.cos(theta)
        p_y = y + _DEFAULTS['l'] * math.sin(theta)

        # call the real compute function so outputs match the ROS node exactly
        v_out, w_out, rho, forces, v_unsm, w_unsm = compute_control_from_state(
            p_x, p_y, theta, target_x, target_y, obs_x, obs_y,
            config={
                'l': _DEFAULTS['l'],
                'k_att': _DEFAULTS['k_att'],
                'k_rep': _DEFAULTS['k_rep'],
                'd_0': _DEFAULTS['d_0'],
                'min_rho': _DEFAULTS['min_rho'],
                'max_rep_force': _DEFAULTS['max_rep_force'],
                'max_v': _DEFAULTS['max_v'],
                'max_w': _DEFAULTS['max_w'],
                'smoothing_alpha': _DEFAULTS['smoothing_alpha'],
                'workspace_limit': _DEFAULTS['workspace_limit'],
                'wall_margin': _DEFAULTS['wall_margin']
            },
            prev_v=v_s, prev_w=w_s
        )

        # use the smoothed outputs as the command applied to the kinematic simulation
        v_s = v_out
        w_s = w_out

        # kinematics (unicycle)
        x = x + v_s * math.cos(theta) * args.dt
        y = y + v_s * math.sin(theta) * args.dt
        theta = theta + w_s * args.dt

        dist_to_target = math.hypot(target_x - p_x, target_y - p_y)

        # row: time, x, y, theta, p_x, p_y, target_x, target_y, dist_to_target, obs_x, obs_y, rho, v, w
        rows.append([round(t,3), x, y, theta, p_x, p_y, target_x, target_y, dist_to_target, obs_x, obs_y, rho, v_s, w_s])

        t += args.dt

    out_file = os.path.join(args.outdir, f'run_log_robot1_{ti+1:02d}.csv')
    with open(out_file, 'w') as fh:
        for r in rows:
            fh.write(','.join(map(str, r)) + '\n')
    print('Wrote', out_file)

print('Simulation complete. CSVs are in', args.outdir)
