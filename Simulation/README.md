Robohub/Simulation - Simulation-only files

Contents:
- hockey_node.py  : Controller node implementing waypoint navigation + potential fields. Supports --use-mock and --log-file.
- mock_vrpn.py    : Lightweight VRPN PoseStamped publisher for testing without the simulator.

Quick demo (single-container, no simulator needed):

1) From your host, run (one line):

docker run --rm -it --network=host --volume /mnt/d/Amir/University_of_Waterloo/Year_4/4A/ECE_486/Project_Simulation:/project_code:rw --env DISPLAY=:0 dji_robomaster_ros:1.0 \
  bash -lc "source /opt/ros/humble/setup.bash && cd /project_code && python3 hockey_node.py --robot 1 --obstacle 2 --use-mock --log-file /project_code/run_log_robot1.csv"

2) After running for a while, stop with Ctrl+C and inspect the CSV:

  tail -n 20 /mnt/d/Amir/University_of_Waterloo/Year_4/4A/ECE_486/Project_Simulation/run_log_robot1.csv

If you want to run with the real simulator, ensure both nodes (simulator and controller) use the same ROS 2 RMW implementation (Cyclone DDS) and that the controller environment has librmw_cyclonedds_cpp installed. See project notes for troubleshooting DDS discovery.
