Simulation — ECE 486 demo (improved README)

Overview
- This README expands the quick demo instructions and explains how to reproduce the results used in the final report.

Files in this folder
- hockey_node.py : Controller node implementing look-ahead approximate linearization + artificial potential fields. Supports --use-mock, detailed logging (--log-file), and configurable parameters via CLI.
- mock_vrpn.py   : Lightweight VRPN PoseStamped publisher for testing without the simulator.

Quick Docker demo (single-command)
- Run the controller with synthetic VRPN (no simulator required). Update the host path as appropriate.

  docker run --rm -it --network=host \
    --volume /mnt/d/Amir/University_of_Waterloo/Year_4/4A/ECE_486/Project_Simulation:/project_code:rw \
    --env DISPLAY=:0 dji_robomaster_ros:1.0 \
    bash -lc "source /opt/ros/humble/setup.bash && cd /project_code && python3 hockey_node.py --robot 1 --obstacle 2 --use-mock --log-file /project_code/run_log_robot1.csv"

What to expect
- While running the demo you will see INFO logs like:
  [INFO]: Ctrl Pt: (x, y) | Tgt: 0.65m | Obs: 1.20m | cmds -> v: 0.48, w: 1.23
- When you stop with Ctrl+C a CSV file will be left at the path given to --log-file. Example CSV fields:
  time, x, y, theta, p_x, p_y, target_x, target_y, dist_to_target, obs_x, obs_y, rho, v, w

Running locally (without Docker)
- If you have ROS 2 Humble and rclpy installed on your machine, run:

  # Terminal 1: mock VRPN
  python3 mock_vrpn.py --robot 1 --rate 30

  # Terminal 2: controller
  python3 hockey_node.py --robot 1 --obstacle 2 --log-file run_log_robot1.csv

Reproducing experiments and plots (recommended steps)
1. Run N trials with different initial positions using the --seed or CLI-start options (I can add this automation script).
2. Collect CSVs into results/ and run an analysis script to compute metrics and generate plots.
3. Include trajectory overlays and metric plots in the final report.

Troubleshooting notes
- If you want to connect to the original simulator instead of the mock, ensure both processes use the same RMW implementation (Cyclone DDS). The simulator uses Cyclone DDS; install and configure the same on the controller container. Use --network=host for cross-container discovery.
- If ROS 2 topics don't show up, check "ros2 topic list" and ensure the namespace and topic names match (vrpn_mocap/dji_robot_<ID>/pose).

Next steps and optional additions
- I can add run_demo.sh to automate Docker runs and produce numbered CSVs per trial.
- I can provide an analysis notebook that reads CSVs and generates the recommended plots and metrics.

Contact
- For any adjustments (change default parameters, add automated trials, or produce a PDF report), reply here and I will continue the work.
