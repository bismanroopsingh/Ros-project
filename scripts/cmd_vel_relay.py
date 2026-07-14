#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self.callback, 10)
        self.pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.get_logger().info('cmd_vel relay started')

    def callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()