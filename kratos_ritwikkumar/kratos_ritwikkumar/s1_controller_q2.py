#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class S1Controller(Node):
    def __init__(self):
        super().__init__('s1_controller')
        self.publisher_ = self.create_publisher(String, '/s1', 10)
        self.state = 'green'
        self.publish_state()

        # Change state every 10 seconds
        self.timer = self.create_timer(10.0, self.timer_callback)

    def timer_callback(self):
        self.state = 'red' if self.state == 'green' else 'green'
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published on /s1: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = S1Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
