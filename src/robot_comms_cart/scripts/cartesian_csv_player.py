#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv

from geometry_msgs.msg import Pose

class CartesianCSVPlayer(Node):
    def __init__(self):
        super().__init__('cartesian_csv_player')

        self.cmd_pub = self.create_publisher(Pose, 'cartesian_command', 10)
        self.command_queue = []
        self.timer = self.create_timer(6.0, self.loop)  # 3 seconds between commands

        self.load_csv('/home/adam/Desktop/cartesianTest.csv')

    def load_csv(self, path):
        try:
            with open(path, newline='') as csvfile:
                reader = csv.reader(csvfile)
                for row in reader:
                    if len(row) != 6:
                        self.get_logger().warn(f"Skipping invalid row: {row}")
                        continue
                    x, y, z, a, b, c = map(float, row)
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z
                    pose.orientation.x = a
                    pose.orientation.y = b
                    pose.orientation.z = c
                    pose.orientation.w = 0.0  # Not used
                    self.command_queue.append(pose)
            self.get_logger().info(f"Loaded {len(self.command_queue)} pose commands.")
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {e}")

    def loop(self):
        if self.command_queue:
            next_pose = self.command_queue.pop(0)
            self.cmd_pub.publish(next_pose)
            self.get_logger().info(f"Sent pose: {next_pose}")
        else:
            self.get_logger().info("All poses sent. Shutting down node.")
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CartesianCSVPlayer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
