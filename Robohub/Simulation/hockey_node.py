import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import qos_profile_sensor_data
import math
import argparse
from enum import Enum

class State(Enum):
    NAVIGATING = 1
    DONE = 2

class SimulatorPlayerNode(Node):
    def __init__(self, robot_id, obstacle_id, use_mock=False, log_file=None):
        super().__init__('sim_player_node')

        self.use_mock = use_mock
        # Publisher for commands (always create so mock mode can publish too)
        self.cmd_vel_pub = self.create_publisher(Twist, f'/robot{robot_id}/cmd_vel', 10)

        # If not in mock mode, subscribe to VRPN topics
        if not self.use_mock:
            # Use Sensor Data QoS (Best Effort) to match motion capture streams!
            self.pose_sub = self.create_subscription(
                PoseStamped, f'/vrpn_mocap/dji_robot_{robot_id}/pose', self.pose_callback, qos_profile_sensor_data)

            self.obs_sub = self.create_subscription(
                PoseStamped, f'/vrpn_mocap/dji_robot_{obstacle_id}/pose', self.obs_callback, qos_profile_sensor_data)
        else:
            # Mock state for synthetic poses when running without simulator
            self._mock_t = 0.0
            self._mock_dt = 0.05
            self.obs_x = 0.8
            self.obs_y = 0.0
            # timer to drive mock poses
            self.mock_timer = self.create_timer(self._mock_dt, self._mock_tick)

        self.l = 0.3

        # safety & tuning
        self.max_v = 0.8       # m/s cap for linear velocity
        self.max_w = 3.0       # rad/s cap for angular velocity
        self.min_rho = 0.12    # minimum obstacle distance to avoid singularity (m)
        self.max_rep_force = 2.0  # cap repulsive force magnitude
        # smoothing state for published commands
        self._prev_v = 0.0
        self._prev_w = 0.0
        self._smoothing_alpha = 0.4  # 0..1, higher = more responsive

        # If mock mode, ensure obs_x/obs_y exist; otherwise will be set by obs_callback
        if not self.use_mock:
            self.obs_x = None
            self.obs_y = None

        self.waypoints = [(1.0, 1.0), (-1.0, 1.0), (0.0, 0.0)]
        self.current_waypoint_idx = 0
        self.log_counter = 0

        # Diagnostic tracker
        # In mock mode we already have synthetic data
        self.received_data = True if self.use_mock else False
        self.diagnostic_timer = self.create_timer(2.0, self.check_connection)

        self.state = State.NAVIGATING

        # Setup logging to CSV if requested
        self.log_file = log_file
        if self.log_file is None:
            self.log_file = f"hockey_run_log_robot{robot_id}.csv"
        try:
            self._log_fh = open(self.log_file, 'w')
            self._log_fh.write('time,x,y,theta,p_x,p_y,target_x,target_y,dist_to_target,obs_x,obs_y,rho,v,w\n')
            self._log_fh.flush()
        except Exception:
            self._log_fh = None

        self.get_logger().info(f"Simulator Node Started! Robot {robot_id} avoiding Robot {obstacle_id} (mock={self.use_mock})")

    def check_connection(self):
        if not self.received_data:
            self.get_logger().warn("Still waiting for /vrpn_mocap data... Is the simulator broadcasting?")

    def _mock_tick(self):
        # Advance a simple simulated pose and obstacle for offline testing
        self._mock_t += self._mock_dt
        t = self._mock_t
        # robot moves slowly on a circular trajectory
        x = 0.5 * math.cos(0.2 * t)
        y = 0.5 * math.sin(0.2 * t)
        theta = 0.2 * t
        # obstacle moves on a different circle
        self.obs_x = 0.8 * math.cos(0.5 * t)
        self.obs_y = 0.8 * math.sin(0.5 * t)
        # call the same control law with the synthetic pose
        self.apply_control_law(x, y, theta)

    def pose_callback(self, msg):
        self.received_data = True  # We got data!
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.apply_control_law(x, y, theta)

    def obs_callback(self, msg):
        self.obs_x = msg.pose.position.x
        self.obs_y = msg.pose.position.y

    def apply_control_law(self, x, y, theta):
        # Only stop if we're fully done
        if self.state == State.DONE:
            return

        # compute look-ahead control point
        p_x = x + self.l * math.cos(theta)
        p_y = y + self.l * math.sin(theta)

        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        dist_to_target = math.sqrt((target_x - p_x)**2 + (target_y - p_y)**2)

        if dist_to_target < 0.1:
            self.get_logger().info(f"*** REACHED WAYPOINT {self.current_waypoint_idx + 1} ***")
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints reached. Patrol complete.")
                self.stop_robot()
                self.state = State.DONE
                return
            else:
                target_x, target_y = self.waypoints[self.current_waypoint_idx]

        # Artificial Potential Fields
        # Gain constants
        k_att = 0.8
        k_rep = 0.3
        d_0 = 0.8  # Influence distance

        # Attractive force
        F_att_x = k_att * (target_x - p_x)
        F_att_y = k_att * (target_y - p_y)

        # Repulsive force: if obstacle not known, skip repulsion
        F_rep_x = 0.0
        F_rep_y = 0.0
        rho = float('inf')
        if self.obs_x is not None and self.obs_y is not None:
            dist_sq = (self.obs_x - p_x)**2 + (self.obs_y - p_y)**2
            rho = math.sqrt(dist_sq)
            # prevent singularity by enforcing a minimum rho
            if rho < self.min_rho:
                rho = self.min_rho
                dist_sq = rho * rho
            if rho < d_0 and rho > 0.0:
                # repulsive magnitude (guarded and clamped)
                raw_force_mag = k_rep * (1.0/rho - 1.0/d_0) * (1.0/dist_sq)
                # clamp extreme values
                force_mag = max(-self.max_rep_force, min(self.max_rep_force, raw_force_mag))
                # direction from obstacle to control point (normalized)
                dx = p_x - self.obs_x
                dy = p_y - self.obs_y
                norm = math.hypot(dx, dy) if (dx != 0.0 or dy != 0.0) else 1.0
                F_rep_x = force_mag * (dx / norm)
                F_rep_y = force_mag * (dy / norm)

        p_dot_x = F_att_x + F_rep_x
        p_dot_y = F_att_y + F_rep_y

        # Approximate Linearization
        v = math.cos(theta) * p_dot_x + math.sin(theta) * p_dot_y
        w = (-math.sin(theta) * p_dot_x + math.cos(theta) * p_dot_y) / self.l

        # Saturate velocities to safe limits
        if v > self.max_v:
            v = self.max_v
        elif v < -self.max_v:
            v = -self.max_v
        if w > self.max_w:
            w = self.max_w
        elif w < -self.max_w:
            w = -self.max_w

        # Exponential smoothing to reduce spikes
        try:
            v = self._prev_v * (1.0 - self._smoothing_alpha) + v * self._smoothing_alpha
            w = self._prev_w * (1.0 - self._smoothing_alpha) + w * self._smoothing_alpha
            self._prev_v = v
            self._prev_w = w
        except Exception:
            # if smoothing state missing for any reason, skip smoothing
            pass

        self.log_counter += 1
        if self.log_counter % 5 == 0:
            self.get_logger().info(
                f"Ctrl Pt: ({p_x:.2f}, {p_y:.2f}) | Tgt: {dist_to_target:.2f}m | Obs: {rho:.2f}m | cmds -> v: {v:.2f}, w: {w:.2f}"
            )

        # Publish command
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = w
        self.cmd_vel_pub.publish(twist_msg)

        # Write telemetry log if requested
        try:
            if getattr(self, '_log_fh', None):
                import time
                tnow = time.time()
                obs_x = self.obs_x if self.obs_x is not None else float('nan')
                obs_y = self.obs_y if self.obs_y is not None else float('nan')
                line = f"{tnow:.3f},{x:.4f},{y:.4f},{theta:.4f},{p_x:.4f},{p_y:.4f},{target_x:.4f},{target_y:.4f},{dist_to_target:.4f},{obs_x:.4f},{obs_y:.4f},{rho:.4f},{v:.4f},{w:.4f}\n"
                self._log_fh.write(line)
                self._log_fh.flush()
        except Exception:
            pass

    def stop_robot(self):
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=int, default=1)
    parser.add_argument('--obstacle', type=int, default=2)
    parser.add_argument('--use-mock', action='store_true', help='Run controller with synthetic VRPN poses (no simulator required)')
    parser.add_argument('--log-file', type=str, default=None, help='Path to CSV log file to record telemetry')
    custom_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SimulatorPlayerNode(custom_args.robot, custom_args.obstacle, use_mock=custom_args.use_mock, log_file=custom_args.log_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if getattr(node, '_log_fh', None):
                node._log_fh.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # ignore rclpy shutdown errors (e.g., already shutdown from signal)
            pass

if __name__ == '__main__':
    main()
