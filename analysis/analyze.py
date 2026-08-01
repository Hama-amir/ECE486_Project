#!/usr/bin/env python3
"""
analyze.py
Reads CSVs in ../results, computes metrics and outputs plots into ../results/plots
"""
import glob
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(__file__))
RESULTS = os.path.join(ROOT, 'results')
PLOTS = os.path.join(RESULTS, 'plots')
os.makedirs(PLOTS, exist_ok=True)

files = sorted(glob.glob(os.path.join(RESULTS, 'run_log_robot1_*.csv')))
if not files:
    print('No CSV files found in', RESULTS)
    exit(1)

metrics = []
threshold_goal = 0.20  # meters
collision_threshold = 0.12  # meters

for f in files:
    data = np.loadtxt(f, delimiter=',')
    # expected columns: time, x, y, theta, p_x, p_y, target_x, target_y, dist_to_target, obs_x, obs_y, rho, v, w
    t = data[:,0]
    x = data[:,1]
    y = data[:,2]
    dist_to_target = data[:,8]
    obs_x = data[:,9]
    obs_y = data[:,10]
    rho = data[:,11]
    v = data[:,12]
    w = data[:,13]

    # success
    reached_idx = np.where(dist_to_target < threshold_goal)[0]
    success = reached_idx.size > 0
    time_to_goal = t[reached_idx[0]] if success else t[-1]

    # collisions
    min_rho = np.min(rho)
    collision = min_rho < collision_threshold

    # path length
    dx = np.diff(x)
    dy = np.diff(y)
    path_length = np.sum(np.sqrt(dx*dx + dy*dy))

    avg_rho = np.mean(rho)

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

# aggregate plots: overlay trajectories
plt.figure(figsize=(6,6))
for f in files:
    data = np.loadtxt(f, delimiter=',')
    x = data[:,1]
    y = data[:,2]
    obs_x = data[:,9]
    obs_y = data[:,10]
    plt.plot(x, y, alpha=0.7)
    plt.plot(obs_x, obs_y, '--', alpha=0.6)
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('Trajectories (agent solid, obstacle dashed)')
plt.xlim(-2.2, 2.2)
plt.ylim(-2.2, 2.2)
plt.grid(True)
plt.savefig(os.path.join(PLOTS, 'trajectories.png'))
plt.close()

# distance to obstacle over time for first trial
data = np.loadtxt(files[0], delimiter=',')
t = data[:,0]
rho = data[:,11]
plt.figure()
plt.plot(t, rho)
plt.xlabel('time (s)')
plt.ylabel('rho (m)')
plt.title('Distance to obstacle (trial 1)')
plt.grid(True)
plt.savefig(os.path.join(PLOTS, 'rho_trial1.png'))
plt.close()

# v and w over time for first trial
v = data[:,12]
w = data[:,13]
plt.figure()
plt.plot(t, v, label='v')
plt.plot(t, w, label='w')
plt.xlabel('time (s)')
plt.legend()
plt.title('v and w (trial 1)')
plt.grid(True)
plt.savefig(os.path.join(PLOTS, 'vw_trial1.png'))
plt.close()

print('Plots written to', PLOTS)
print('Metrics per trial:')
for m in metrics:
    print(m)
