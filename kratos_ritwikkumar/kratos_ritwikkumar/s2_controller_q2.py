#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class S2Controller(Node):
    def __init__(self):
        super().__init__('s2_controller')
        self.subscription = self.create_subscription(
            String,
            '/s1',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/s2', 10)

    def listener_callback(self, msg):
        s1_state = msg.data
        s2_state = 'red' if s1_state == 'green' else 'green'

        out_msg = String()
        out_msg.data = s2_state
        self.publisher_.publish(out_msg)

        self.get_logger().info(f'/s1: {s1_state} therefore /s2: {s2_state}')

def main(args=None):
    rclpy.init(args=args)
    node = S2Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
