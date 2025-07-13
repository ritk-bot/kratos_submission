#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

class FKPublisherNode(Node):
    def __init__(self):
        super().__init__('fk_publisher_node')

        # Link lengths
        self.L1 = 2.0  # meters
        self.L2 = 1.5  # meters

        # Subscriber to /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # Publisher to /end_effector_position
        self.publisher_ = self.create_publisher(Point, '/end_effector_position', 10)

        self.get_logger().info("FK Publisher Node has started.")

    def joint_states_callback(self, msg: JointState):
        try:
            # Extract joint angles
            theta1 = msg.position[0] + math.pi / 2  # Adjusted by pi/2 as specified
            theta2 = msg.position[1]

            # Forward Kinematics
            x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
            y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)

            # Publish position
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0  # unused
            self.publisher_.publish(point)

            self.get_logger().info(f"Published End Effector Position: x={x:.2f}, y={y:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error in joint_states_callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = FKPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
