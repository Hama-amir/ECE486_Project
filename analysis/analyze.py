#!/usr/bin/env python3
"""
analyze.py
Reads CSVs in ../results, computes metrics and outputs plots into ../results/plots
"""
import glob
import os
import csv
import math

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

ROOT = os.path.dirname(os.path.dirname(__file__))
RESULTS = os.path.join(ROOT, 'results')
PLOTS = os.path.join(RESULTS, 'plots')
if HAVE_MPL:
    os.makedirs(PLOTS, exist_ok=True)

files = sorted(glob.glob(os.path.join(RESULTS, 'run_log_robot1_*.csv')))
if not files:
    print('No CSV files found in', RESULTS)
    exit(1)

metrics = []
threshold_goal = 0.20  # meters
collision_threshold = 0.12  # meters

for f in files:
    times = []
    xs = []
    ys = []
    dist_to_target = []
    obs_xs = []
    obs_ys = []
    rhos = []
    vs = []
    ws = []

    with open(f, 'r') as fh:
        reader = csv.reader(fh)
        for row in reader:
            if not row:
                continue
            vals = list(map(float, row))
            times.append(vals[0])
            xs.append(vals[1])
            ys.append(vals[2])
            dist_to_target.append(vals[8])
            obs_xs.append(vals[9])
            obs_ys.append(vals[10])
            rhos.append(vals[11])
            vs.append(vals[12])
            ws.append(vals[13])

    # success
    success = any(d < threshold_goal for d in dist_to_target)
    time_to_goal = next((times[i] for i,d in enumerate(dist_to_target) if d < threshold_goal), times[-1])

    # collisions
    min_rho = min(rhos)
    collision = min_rho < collision_threshold

    # path length
    path_length = 0.0
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i-1]
        dy = ys[i] - ys[i-1]
        path_length += math.hypot(dx, dy)

    avg_rho = sum(rhos)/len(rhos)

    metrics.append({
        'file': os.path.basename(f),
        'success': success,
        'time_to_goal': float(time_to_goal),
        'collision': collision,
        'min_rho': float(min_rho),
        'path_length': float(path_length),
        'avg_rho': float(avg_rho)
    })

# write metrics summary
summary_file = os.path.join(RESULTS, 'metrics_summary.csv')
with open(summary_file, 'w') as fh:
    fh.write('file,success,time_to_goal,collision,min_rho,path_length,avg_rho\n')
    for m in metrics:
        fh.write(f"{m['file']},{int(m['success'])},{m['time_to_goal']},{int(m['collision'])},{m['min_rho']},{m['path_length']},{m['avg_rho']}\n")

print('Wrote metrics summary to', summary_file)

if HAVE_MPL:
    # aggregate plots: overlay trajectories
    plt.figure(figsize=(6,6))
    for f in files:
        xs = []
        ys = []
        obs_xs = []
        obs_ys = []
        with open(f, 'r') as fh:
            reader = csv.reader(fh)
            for row in reader:
                if not row:
                    continue
                vals = list(map(float, row))
                xs.append(vals[1])
                ys.append(vals[2])
                obs_xs.append(vals[9])
                obs_ys.append(vals[10])
        plt.plot(xs, ys, alpha=0.7)
        plt.plot(obs_xs, obs_ys, '--', alpha=0.6)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Trajectories (agent solid, obstacle dashed)')
    plt.xlim(-2.2, 2.2)
    plt.ylim(-2.2, 2.2)
    plt.grid(True)
    plt.savefig(os.path.join(PLOTS, 'trajectories.png'))
    plt.close()

    # distance to obstacle over time for first trial
    times = []
    rhos = []
    with open(files[0], 'r') as fh:
        reader = csv.reader(fh)
        for row in reader:
            if not row: continue
            vals = list(map(float, row))
            times.append(vals[0])
            rhos.append(vals[11])
    plt.figure()
    plt.plot(times, rhos)
    plt.xlabel('time (s)')
    plt.ylabel('rho (m)')
    plt.title('Distance to obstacle (trial 1)')
    plt.grid(True)
    plt.savefig(os.path.join(PLOTS, 'rho_trial1.png'))
    plt.close()

    # v and w over time for first trial
    times = []
    vs = []
    ws = []
    with open(files[0], 'r') as fh:
        reader = csv.reader(fh)
        for row in reader:
            if not row: continue
            vals = list(map(float, row))
            times.append(vals[0])
            vs.append(vals[12])
            ws.append(vals[13])
    plt.figure()
    plt.plot(times, vs, label='v')
    plt.plot(times, ws, label='w')
    plt.xlabel('time (s)')
    plt.legend()
    plt.title('v and w (trial 1)')
    plt.grid(True)
    plt.savefig(os.path.join(PLOTS, 'vw_trial1.png'))
    plt.close()

    print('Plots written to', PLOTS)
else:
    print('matplotlib not available; skipping plots')

print('Metrics per trial:')
for m in metrics:
    print(m)
