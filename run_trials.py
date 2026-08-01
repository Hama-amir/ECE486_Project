#!/usr/bin/env python3
"""
run_trials.py
Run multiple headless trials using the Docker image and save CSV logs into results/.
Usage: python3 run_trials.py --n 10 --duration 12
"""
import argparse
import os
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--n', '-n', type=int, default=10, help='Number of trials')
parser.add_argument('--duration', '-d', type=int, default=12, help='Duration per trial in seconds')
parser.add_argument('--project-path', type=str, default=os.getcwd(), help='Host path mounted into the container (project root)')
parser.add_argument('--image', type=str, default='dji_robomaster_ros:1.0', help='Docker image to use')
args = parser.parse_args()

results_dir = os.path.join(args.project_path, 'results')
os.makedirs(results_dir, exist_ok=True)

for i in range(args.n):
    out_csv = f'/project_code/results/run_log_robot1_{i+1:02d}.csv'
    host_out = os.path.join(results_dir, f'run_log_robot1_{i+1:02d}.csv')
    print(f'Trial {i+1}/{args.n}: output -> {host_out}')
    cmd = (
        'docker run --rm -it --network=host '
        f"--volume {args.project_path}:/project_code:rw --env DISPLAY=:0 {args.image} "
        f"bash -lc \"source /opt/ros/humble/setup.bash && cd /project_code && timeout {args.duration}s python3 hockey_node.py --robot 1 --obstacle 2 --use-mock --log-file {out_csv}\""
    )
    try:
        subprocess.run(cmd, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f'Warning: trial {i+1} failed: {e}', file=sys.stderr)
        # proceed to next trial

print('All trials completed. CSVs are in:', results_dir)
