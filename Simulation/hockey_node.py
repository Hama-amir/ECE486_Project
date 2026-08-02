# rclpy and ROS message imports are optional for headless import by the evaluator
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    HAVE_RCLPY = True
except Exception:
    # allow importing compute_control_from_state in environments without rclpy
    HAVE_RCLPY = False
    class Node:  # simple placeholder so the SimulatorPlayerNode class can be defined but not used
        pass
    qos_profile_sensor_data = None

try:
    from geometry_msgs.msg import Twist, PoseStamped
except Exception:
    # placeholders when ROS message packages are not available (headless eval path)
    class Twist:
        def __init__(self):
            self.linear = type('L', (), {'x': 0.0})()
            self.angular = type('A', (), {'z': 0.0})()
    class PoseStamped:
        pass

import math
import argparse
import time
from enum import Enum

# Module-level defaults (used when imported by headless evaluator)
_DEFAULTS = {
    'l': 0.3,
    'k_att': 1.0,
    'k_rep': 0.5,
    'd_0': 1.0,
    'min_rho': 0.12,
    'max_rep_force': 2.0,
    'max_v': 0.8,
    'max_w': 3.0,
    'smoothing_alpha': 0.4,
    'workspace_limit': 2.0,
    'wall_margin': 0.2,
    'obs_stale_timeout': 0.5  # seconds
}

class State(Enum):
    NAVIGATING = 1
    DONE = 2


def compute_control_from_state(p_x, p_y, theta, target_x, target_y, obs_x, obs_y, config=None, prev_v=0.0, prev_w=0.0):
    """
    Pure function computing smoothed, saturated (v,w) given the look-ahead control point p=(p_x,p_y),
    robot heading theta, target and obstacle positions. `config` can override defaults.
    Returns (v_out, w_out, rho, raw_forces, v_unsmoothed, w_unsmoothed)
    """
    cfg = dict(_DEFAULTS)
    if config:
        cfg.update(config)

    # Attractive
    k_att = cfg['k_att']
    k_rep = cfg['k_rep']
    d_0 = cfg['d_0']

    F_att_x = k_att * (target_x - p_x)
    F_att_y = k_att * (target_y - p_y)

    # Repulsive from obstacle
    F_rep_x = 0.0
    F_rep_y = 0.0
    rho = float('inf')
    if obs_x is not None and obs_y is not None:
        dx = p_x - obs_x
        dy = p_y - obs_y
        dist_sq = dx*dx + dy*dy
        rho = math.sqrt(dist_sq)
        if rho < cfg['min_rho']:
            rho = cfg['min_rho']
            dist_sq = rho*rho
        if rho < d_0 and rho > 0.0:
            raw_force_mag = k_rep * (1.0/rho - 1.0/d_0) * (1.0/dist_sq)
            force_mag = max(-cfg['max_rep_force'], min(cfg['max_rep_force'], raw_force_mag))
            norm = math.hypot(dx, dy) if (dx != 0.0 or dy != 0.0) else 1.0
            F_rep_x = force_mag * (dx / norm)
            F_rep_y = force_mag * (dy / norm)

    # Wall repulsion to keep inside workspace
    # left/right walls at x = +/- workspace_limit, bottom/top at y = +/- workspace_limit
    wall_margin = cfg['wall_margin']
    Wx = 0.0
    Wy = 0.0
    # right wall
    wl = cfg['workspace_limit']
    # distance from right wall = wl - p_x
    d_right = wl - p_x
    if d_right < wall_margin:
        # repulse to the left
        mag = cfg['k_rep'] * (1.0/max(d_right,1e-3) - 1.0/wall_margin) * (1.0/(max(d_right,1e-3)**2))
        mag = max(-cfg['max_rep_force'], min(cfg['max_rep_force'], mag))
        Wx -= mag
    # left wall
    d_left = p_x + wl
    if d_left < wall_margin:
        mag = cfg['k_rep'] * (1.0/max(d_left,1e-3) - 1.0/wall_margin) * (1.0/(max(d_left,1e-3)**2))
        mag = max(-cfg['max_rep_force'], min(cfg['max_rep_force'], mag))
        Wx += mag
    # top wall
    d_top = wl - p_y
    if d_top < wall_margin:
        mag = cfg['k_rep'] * (1.0/max(d_top,1e-3) - 1.0/wall_margin) * (1.0/(max(d_top,1e-3)**2))
        mag = max(-cfg['max_rep_force'], min(cfg['max_rep_force'], mag))
        Wy -= mag
    # bottom wall
    d_bottom = p_y + wl
    if d_bottom < wall_margin:
        mag = cfg['k_rep'] * (1.0/max(d_bottom,1e-3) - 1.0/wall_margin) * (1.0/(max(d_bottom,1e-3)**2))
        mag = max(-cfg['max_rep_force'], min(cfg['max_rep_force'], mag))
        Wy += mag

    F_rep_x += Wx
    F_rep_y += Wy

    p_dot_x = F_att_x + F_rep_x
    p_dot_y = F_att_y + F_rep_y

    # approximate linearization
    l = cfg['l']
    v_unsm = math.cos(theta) * p_dot_x + math.sin(theta) * p_dot_y
    w_unsm = (-math.sin(theta) * p_dot_x + math.cos(theta) * p_dot_y) / l

    # saturate
    v = max(-cfg['max_v'], min(cfg['max_v'], v_unsm))
    w = max(-cfg['max_w'], min(cfg['max_w'], w_unsm))

    # smoothing
    alpha = cfg['smoothing_alpha']
    v_out = alpha * v + (1 - alpha) * prev_v
    w_out = alpha * w + (1 - alpha) * prev_w

    return v_out, w_out, rho, (F_att_x, F_att_y, F_rep_x, F_rep_y), v_unsm, w_unsm


class SimulatorPlayerNode(Node):
    def __init__(self, robot_id, obstacle_id, use_mock=False, log_file=None):
        super().__init__('sim_player_node')

        self.use_mock = use_mock
        # Publisher for commands (always create so mock mode can publish too)
        self.cmd_vel_pub = self.create_publisher(Twist, f'/robot{robot_id}/cmd_vel', 10)

        # Parameters (declared so they can be tuned at launch)
        self.declare_parameter('l', _DEFAULTS['l'])
        self.declare_parameter('k_att', _DEFAULTS['k_att'])
        self.declare_parameter('k_rep', _DEFAULTS['k_rep'])
        self.declare_parameter('d_0', _DEFAULTS['d_0'])
        self.declare_parameter('min_rho', _DEFAULTS['min_rho'])
        self.declare_parameter('max_rep_force', _DEFAULTS['max_rep_force'])
        self.declare_parameter('max_v', _DEFAULTS['max_v'])
        self.declare_parameter('max_w', _DEFAULTS['max_w'])
        self.declare_parameter('smoothing_alpha', _DEFAULTS['smoothing_alpha'])
        self.declare_parameter('workspace_limit', _DEFAULTS['workspace_limit'])
        self.declare_parameter('wall_margin', _DEFAULTS['wall_margin'])
        self.declare_parameter('obs_stale_timeout', _DEFAULTS['obs_stale_timeout'])

        # Obstacle pose state — initialized unconditionally so it always exists,
        # regardless of whether we're in mock or real (VRPN) mode.
        self.obs_x = None
        self.obs_y = None
        self.obs_ts = None

        # read params into instance fields for use by ROS node
        self.l = float(self.get_parameter('l').value)
        self._param_k_att = float(self.get_parameter('k_att').value)
        self._param_k_rep = float(self.get_parameter('k_rep').value)
        self._param_d_0 = float(self.get_parameter('d_0').value)
        self.min_rho = float(self.get_parameter('min_rho').value)
        self.max_rep_force = float(self.get_parameter('max_rep_force').value)
        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self._smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self._workspace_limit = float(self.get_parameter('workspace_limit').value)
        self._wall_margin = float(self.get_parameter('wall_margin').value)
        self._obs_stale_timeout = float(self.get_parameter('obs_stale_timeout').value)

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
            self.obs_ts = time.time()
            # timer to drive mock poses
            self.mock_timer = self.create_timer(self._mock_dt, self._mock_tick)

        # smoothing state for published commands
        self._prev_v = 0.0
        self._prev_w = 0.0

        self.waypoints = [(1.0, 1.0), (-1.0, 1.0), (0.0, 0.0)]
        self.current_waypoint_idx = 0

        # Diagnostic tracker
        # In mock mode we already have synthetic data
        self.received_data = True if self.use_mock else False
        self.diagnostic_timer = self.create_timer(2.0, self.check_connection)

        self.state = State.NAVIGATING

        # logging throttle state (use timestamps rather than counter)
        self._last_log_time = 0.0
        self._log_throttle = 0.5  # seconds

        # Setup logging to CSV if requested
        self.log_file = log_file
        if self.log_file is None:
            self.log_file = f"hockey_run_log_robot{robot_id}.csv"
        try:
            self._log_fh = open(self.log_file, 'w')
            self._log_fh.write('time,x,y,theta,p_x,p_y,target_x,target_y,dist_to_target,obs_x,obs_y,rho,v,w\n')
            self._log_fh.flush()
        except Exception as e:
            self.get_logger().warning(f"Unable to open log file {self.log_file}: {e}")
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
        self.obs_ts = time.time()
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
        self.obs_ts = time.time()

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

        # check obstacle recency
        obs_x = None
        obs_y = None
        if self.obs_ts is not None:
            if time.time() - self.obs_ts < self._obs_stale_timeout:
                obs_x = self.obs_x
                obs_y = self.obs_y
            else:
                # stale obstacle data
                self.get_logger().warning('Obstacle pose data is stale; ignoring obstacle until updated')
        # else: no obstacle pose has ever been received — obs_x/obs_y remain None,
        # and compute_control_from_state() handles that by skipping repulsion.

        # Use module-level compute function so external evaluators can import and reproduce exact behavior
        config = {
            'l': self.l,
            'k_att': self._param_k_att,
            'k_rep': self._param_k_rep,
            'd_0': self._param_d_0,
            'min_rho': self.min_rho,
            'max_rep_force': self.max_rep_force,
            'max_v': self.max_v,
            'max_w': self.max_w,
            'smoothing_alpha': self._smoothing_alpha,
            'workspace_limit': self._workspace_limit,
            'wall_margin': self._wall_margin
        }

        v, w, rho, forces, v_unsm, w_unsm = compute_control_from_state(
            p_x, p_y, theta, target_x, target_y, obs_x, obs_y, config=config, prev_v=self._prev_v, prev_w=self._prev_w
        )

        # update previous smoothed values
        self._prev_v = v
        self._prev_w = w

        # logging (throttled by time)
        nowt = time.time()
        if nowt - self._last_log_time > self._log_throttle:
            self.get_logger().info(
                f"Ctrl Pt: ({p_x:.2f}, {p_y:.2f}) | Tgt: {dist_to_target:.2f}m | Obs: {rho:.2f}m | cmds -> v: {v:.2f}, w: {w:.2f}"
            )
            self._last_log_time = nowt

        # Publish command
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = w
        self.cmd_vel_pub.publish(twist_msg)

        # Write telemetry log if requested
        if getattr(self, '_log_fh', None):
            try:
                tnow = time.time()
                obs_x_log = obs_x if obs_x is not None else float('nan')
                obs_y_log = obs_y if obs_y is not None else float('nan')
                line = f"{tnow:.3f},{x:.4f},{y:.4f},{theta:.4f},{p_x:.4f},{p_y:.4f},{target_x:.4f},{target_y:.4f},{dist_to_target:.4f},{obs_x_log:.4f},{obs_y_log:.4f},{rho:.4f},{v:.4f},{w:.4f}\n"
                self._log_fh.write(line)
                self._log_fh.flush()
            except (OSError, IOError) as e:
                self.get_logger().warning(f"Logging write failed: {e}")

    def stop_robot(self):
        try:
            twist_msg = Twist()
            self.cmd_vel_pub.publish(twist_msg)
        except Exception as e:
            self.get_logger().warning(f"Failed to publish stop command: {e}")

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=int, default=1)
    parser.add_argument('--obstacle', type=int, default=2)
    parser.add_argument('--use-mock', action='store_true', help='Run controller with synthetic VRPN poses (no simulator required)')
    parser.add_argument('--log-file', type=str, default=None, help='Path to CSV log file to record telemetry')
    custom_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = None
    try:
        node = SimulatorPlayerNode(custom_args.robot, custom_args.obstacle, use_mock=custom_args.use_mock, log_file=custom_args.log_file)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # log and ensure robot is stopped before exit
        if node is not None:
            node.get_logger().error(f"Unexpected error in node: {e}")
            try:
                node.stop_robot()
            except Exception:
                pass
        raise
    finally:
        # ensure stop is sent on all shutdown paths
        if node is not None:
            try:
                node.stop_robot()
            except Exception:
                pass
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
