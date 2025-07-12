#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from builtin_interfaces.msg import Duration
from mars_rover_msgs.msg import RoverStatus

class RoverStatusPublisher(Node):
    def __init__(self):
        super().__init__('rover_status_publisher')
        self.publisher_ = self.create_publisher(RoverStatus, 'rover_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

        self.elapsed_seconds = 0.0

    def publish_status(self):
        msg = RoverStatus()

        # Example velocity
        msg.velocity.linear.x = 0.5
        msg.velocity.angular.z = 0.1

        # Example distance
        msg.distance_traveled = 12.34

        # Coordinates (Pose)
        msg.coordinates.position.x = 2.0
        msg.coordinates.position.y = 3.5
        msg.coordinates.position.z = 0.0
        msg.coordinates.orientation.w = 1.0

        # Battery level
        msg.battery_level = 76.5

        # Time of travel (Duration)
        self.elapsed_seconds += 1.0
        msg.time_of_travel.sec = int(self.elapsed_seconds)
        msg.time_of_travel.nanosec = int((self.elapsed_seconds % 1.0) * 1e9)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published rover status: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = RoverStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
