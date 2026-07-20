import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
from enum import Enum

# Define our State Machine categories
class State(Enum):
    NAVIGATING_TO_PRE_GRASP = 1
    ALIGNING = 2
    APPROACHING = 3
    GRASPING = 4

class HockeyPlayerNode(Node):
    def __init__(self):
        super().__init__('hockey_player')
        
        # Subscribers for Vicon tracking data
        self.pose_sub = self.create_subscription(
            PoseStamped, '/vrpn_mocap/robot1/pose', self.pose_callback, 10)
        self.stick_sub = self.create_subscription(
            PoseStamped, '/vrpn_mocap/stick/pose', self.stick_callback, 10)
            
        # Publisher for sending velocity commands to the RoboMaster
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/robot1/cmd_vel', 10)
        
        # Look-ahead distance 'l' for approximate linearization
        self.l = 0.3 
        
        # Variables to store the stick's dynamic target coordinates and heading
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        
        # Initialize the State Machine
        self.state = State.NAVIGATING_TO_PRE_GRASP
        
        self.get_logger().info("Hockey Player Node Started!")


    def pose_callback(self, msg):
        # Extract robot's current x and y coordinates
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract the quaternion and convert to a yaw angle (theta)
        q = msg.pose.orientation
        
        # Convert the quaternion to a 2D heading angle (theta)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Run the state machine and control law
        self.apply_control_law(x, y, theta)


    def stick_callback(self, msg):
        # Continuously update the target coordinates whenever Vicon sees the stick
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        
        # Extract quaternion and convert to yaw angle for the stick's orientation
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.target_theta = math.atan2(siny_cosp, cosy_cosp)

    def apply_control_law(self, x, y, theta):
        # SAFETY CHECK: Wait until Vicon has found the stick
        if self.target_x is None or self.target_theta is None:
            return

        # ---------------------------------------------------------
        # STATE 1: NAVIGATING TO PRE-GRASP
        # ---------------------------------------------------------
        if self.state == State.NAVIGATING_TO_PRE_GRASP:
            # Create a point 0.4 meters directly behind the stick
            offset = 0.4
            pre_grasp_x = self.target_x - offset * math.cos(self.target_theta)
            pre_grasp_y = self.target_y - offset * math.sin(self.target_theta)
            
            p_x = x + self.l * math.cos(theta)
            p_y = y + self.l * math.sin(theta)
            
            error_x = pre_grasp_x - p_x
            error_y = pre_grasp_y - p_y
            distance = math.sqrt(error_x**2 + error_y**2)
            
            # TRANSITION CHECK: Are we close to the pre-grasp point?
            if distance < 0.05:
                self.get_logger().info("Reached pre-grasp point. Transitioning to ALIGNING.")
                self.stop_robot()
                self.state = State.ALIGNING
                return
            
            # Approximate Linearization Math
            K = 1.0 
            p_dot_x = K * error_x
            p_dot_y = K * error_y
            
            v = math.cos(theta) * p_dot_x + math.sin(theta) * p_dot_y
            w = (-math.sin(theta) * p_dot_x + math.cos(theta) * p_dot_y) / self.l
            self.publish_twist(v, w)

        # ---------------------------------------------------------
        # STATE 2: ALIGNING
        # ---------------------------------------------------------
        elif self.state == State.ALIGNING:
            # Find the angular error between robot heading and stick heading
            angle_error = self.target_theta - theta
            
            # Normalize the angle to be strictly between -PI and PI
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            
            # TRANSITION CHECK: Are we facing the stick?
            if abs(angle_error) < 0.05:
                self.get_logger().info("Aligned! Transitioning to APPROACHING.")
                self.stop_robot()
                self.state = State.APPROACHING
                return
                
            # Pure rotation math
            K_w = 1.0
            v = 0.0
            w = K_w * angle_error
            self.publish_twist(v, w)

        # ---------------------------------------------------------
        # STATE 3: APPROACHING
        # ---------------------------------------------------------
        elif self.state == State.APPROACHING:
            # Calculate distance from the center of the robot to the stick
            distance_to_stick = math.sqrt((self.target_x - x)**2 + (self.target_y - y)**2)
            
            # TRANSITION CHECK: Is the stick inside the gripper?
            if distance_to_stick < 0.15: 
                self.get_logger().info("Stick in range! Transitioning to GRASPING.")
                self.stop_robot()
                self.state = State.GRASPING
                return
                
            # Pure forward motion
            K_v = 0.5
            v = K_v * distance_to_stick
            w = 0.0
            self.publish_twist(v, w)
            
        # ---------------------------------------------------------
        # STATE 4: GRASPING
        # ---------------------------------------------------------
        elif self.state == State.GRASPING:
            # Placeholder for Task 2 logic
            pass

    # ---------------------------------------------------------
    # Helper Functions
    # ---------------------------------------------------------
    def stop_robot(self):
        self.publish_twist(0.0, 0.0)
        
    def publish_twist(self, v, w):
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = w
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HockeyPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()