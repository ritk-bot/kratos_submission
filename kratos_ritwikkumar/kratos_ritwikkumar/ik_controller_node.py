#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math
import threading

class IKControllerNode(Node):
    def __init__(self):
        super().__init__('ik_controller_node')

        # Link lengths
        self.L1 = 2.0
        self.L2 = 1.5

        # Last known position
        self.current_x = 0.0
        self.current_y = 0.0

        # Subscriber to current end-effector position
        self.subscription = self.create_subscription(
            Point,
            '/end_effector_position',
            self.position_callback,
            10
        )

        # Publisher for goal joint angles
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_angles_goal', 10)

        # Start user input thread
        threading.Thread(target=self.user_input_loop, daemon=True).start()

        self.get_logger().info("IK Controller Node started. Waiting for end-effector position...")

    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y

    def user_input_loop(self):
        while rclpy.ok():
            try:
                direction = input("\nEnter direction to move ('x' or 'y'): ").strip().lower()
                if direction not in ['x', 'y']:
                    print("Invalid direction. Enter 'x' or 'y'.")
                    continue

                distance = float(input("Enter distance to move (max 0.5 m): "))
                if abs(distance) > 0.5:
                    print("Distance too large! Maximum allowed is 0.5 meters.")
                    continue

                # Compute target
                target_x = self.current_x + distance if direction == 'x' else self.current_x
                target_y = self.current_y + distance if direction == 'y' else self.current_y

                self.get_logger().info(f"Target position: x={target_x:.2f}, y={target_y:.2f}")

                # Perform inverse kinematics
                dx = target_x
                dy = target_y
                d = math.hypot(dx, dy)

                if d > (self.L1 + self.L2) or d < abs(self.L1 - self.L2):
                    self.get_logger().warn("Target position unreachable. Skipping...")
                    continue

                # Inverse Kinematics
                cos_theta2 = (dx**2 + dy**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
                sin_theta2 = math.sqrt(1 - cos_theta2**2)  # elbow-down
                theta2 = math.atan2(sin_theta2, cos_theta2)

                k1 = self.L1 + self.L2 * cos_theta2
                k2 = self.L2 * sin_theta2
                theta1 = math.atan2(dy, dx) - math.atan2(k2, k1)

                # Publish joint angles
                msg = Float64MultiArray()
                msg.data = [theta1, theta2]
                self.publisher_.publish(msg)

                self.get_logger().info(f"Published joint angles: θ1={theta1:.2f}, θ2={theta2:.2f}")

            except Exception as e:
                self.get_logger().error(f"Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = IKControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
