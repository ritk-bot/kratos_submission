#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')

        # Time variables
        self.hour = 0
        self.minute = 0
        self.second = 0

        # Publishers
        self.hour_pub = self.create_publisher(Int32, '/hour', 10)
        self.minute_pub = self.create_publisher(Int32, '/minute', 10)
        self.second_pub = self.create_publisher(Int32, '/second', 10)
        self.clock_pub = self.create_publisher(String, '/clock', 10)

        # Timer to call publish_time every second
        self.timer = self.create_timer(1.0, self.publish_time)

    def publish_time(self):
        # Publish second
        self.second_pub.publish(Int32(data=self.second))

        # Check if second reached 60
        if self.second >= 60:
            self.second = 0
            self.minute += 1
            self.minute_pub.publish(Int32(data=self.minute))

            # Check if minute reached 60
            if self.minute >= 60:
                self.minute = 0
                self.hour += 1
                self.hour_pub.publish(Int32(data=self.hour))

        else:
            self.minute_pub.publish(Int32(data=self.minute))
            self.hour_pub.publish(Int32(data=self.hour))

        # Format time string and publish on /clock
        time_string = f'{self.hour:02}:{self.minute:02}:{self.second:02}'
        self.clock_pub.publish(String(data=time_string))

        # Log (optional)
        self.get_logger().info(f'Current Time: {time_string}')

        # Increment seconds
        self.second += 1

def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
