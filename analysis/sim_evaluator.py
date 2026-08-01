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

# controller parameters (same defaults as report)
l = 0.30
zeta = 1.0
eta = 0.5
rho0 = 1.0
min_rho = 0.12
max_rep_force = 2.0
max_v = 0.8
max_w = 3.0
alpha = 0.4  # smoothing

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
        p_x = x + l * math.cos(theta)
        p_y = y + l * math.sin(theta)

        # attractive
        F_att_x = zeta * (target_x - p_x)
        F_att_y = zeta * (target_y - p_y)

        # repulsive
        dvec_x = p_x - obs_x
        dvec_y = p_y - obs_y
        rho = math.hypot(dvec_x, dvec_y)
        if rho < min_rho:
            rho = min_rho
        F_rep_x = 0.0
        F_rep_y = 0.0
        if rho < rho0:
            grad_x = dvec_x / rho
            grad_y = dvec_y / rho
            mag = eta * (1.0 / rho - 1.0 / rho0) * (1.0 / (rho * rho))
            F_rep_x = mag * grad_x
            F_rep_y = mag * grad_y
            # clamp magnitude
            magF = math.hypot(F_rep_x, F_rep_y)
            if magF > max_rep_force:
                scale = max_rep_force / magF
                F_rep_x *= scale
                F_rep_y *= scale

        p_dot_x = F_att_x + F_rep_x
        p_dot_y = F_att_y + F_rep_y

        # approximate linearization
        v = p_dot_x * math.cos(theta) + p_dot_y * math.sin(theta)
        w = (-p_dot_x * math.sin(theta) + p_dot_y * math.cos(theta)) / l

        # saturate
        v = max(-max_v, min(max_v, v))
        w = max(-max_w, min(max_w, w))

        # smoothing
        v_s = alpha * v + (1 - alpha) * v_s
        w_s = alpha * w + (1 - alpha) * w_s

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
