import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class HockeyPlayerNode(Node):
    def __init__(self):
        super().__init__('hockey_player')
        
        # Subscriber: Listen to Vicon mocap data
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/robot1/pose',
            self.pose_callback,
            10
        )
        
        # Publisher: Send velocity commands to the RoboMaster
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/robot1/cmd_vel',
            10
        )
        
        # Look-ahead distance 'l' for approximate linearization
        self.l = 0.3 

        # Variables to store the stick's target coordinates
        self.target_x = None
        self.target_y = None
        
        self.get_logger().info("Hockey Player Node Started!")


    def pose_callback(self, msg):
        # Extract x and y coordinates
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract the quaternion orientation
        q = msg.pose.orientation
        
        # Convert the quaternion to a 2D heading angle (theta)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Pass these values to our control algorithm
        self.apply_control_law(x, y, theta)


    def stick_callback(self, msg):
        # Continuously update the target coordinates whenever Vicon sees the stick
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y


    def apply_control_law(self, x, y, theta):
        # SAFETY CHECK: Do not compute anything if we haven't seen the stick yet
        if self.target_x is None or self.target_y is None:
            self.get_logger().info("Waiting for stick location from Vicon...", throttle_duration_sec=5.0)
            return
        
        # Calculate the current position of the control point p
        p_x = x + self.l * math.cos(theta)
        p_y = y + self.l * math.sin(theta)
        
        # Calculate the desired velocity of point p to reach the target
        K = 1.0 
        p_dot_x = K * (self.target_x - p_x)
        p_dot_y = K * (self.target_y - p_y)
        
        # Approximate Linearization Inverse Math
        v = math.cos(theta) * p_dot_x + math.sin(theta) * p_dot_y
        w = (-math.sin(theta) * p_dot_x + math.cos(theta) * p_dot_y) / self.l
        
        # Send the commands to the robot
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = w
        self.cmd_vel_pub.publish(twist_msg)