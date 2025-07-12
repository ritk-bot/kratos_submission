#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):

    def __init__(self):
        super().__init__('hello_publisher_node')
        self.publisher_ = self.create_publisher(String, '/new', 10)
        timer_period = 1.0 / 15.0  # 15 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World !'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
