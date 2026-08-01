#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math, time

class MockVrpn(Node):
    def __init__(self, robot_id=1, rate_hz=20.0):
        super().__init__('mock_vrpn')
        self.pub = self.create_publisher(PoseStamped, f'/vrpn_mocap/dji_robot_{robot_id}/pose', 10)
        self.timer = self.create_timer(1.0/rate_hz, self.tick)
        self.t0 = time.time()
        self.get_logger().info(f'Publishing mock VRPN on /vrpn_mocap/dji_robot_{robot_id}/pose')

    def tick(self):
        t = time.time() - self.t0
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = 0.5 * math.cos(0.5 * t)
        msg.pose.position.y = 0.5 * math.sin(0.5 * t)
        yaw = 0.5 * t
        half = yaw * 0.5
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockVrpn()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
