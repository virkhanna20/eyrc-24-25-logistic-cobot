#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult as NavigationResult
import math
from geometry_msgs.msg import PoseStamped
import time

class EBotNavigator(Node):
    def __init__(self):
        super().__init__('ebot_nav_cmd')
        self.navigator = BasicNavigator()

        # Define the tolerance limits
        self.pose_tolerance = 0.3
        self.yaw_tolerance = math.radians(10)  # Converting degrees to radians

        # Start navigation
        self.start_navigation()

    def start_navigation(self):
        # Wait for Nav2 to be ready
        self.get_logger().info('Waiting for Nav2 to be ready...')
        self.navigator.waitUntilNav2Active()

        # Define the goal poses
        goals = [
            self.create_goal_pose(-0.12, -2.35, 3.14),  # P1
            self.create_goal_pose(1.86, 2.56, 0.97),   # P2
            self.create_goal_pose(-3.84, 2.64, 2.78)   # P3
        ]

        for i, goal in enumerate(goals, start=1):
            self.get_logger().info(f'Navigating to P{i}...')
            self.navigator.goToPose(goal)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f'Current ETA: {feedback.estimated_time_remaining.sec} seconds')

            result = self.navigator.getResult()

            if result == NavigationResult.SUCCEEDED:
                self.get_logger().info(f'Successfully reached P{i}')
            elif result == NavigationResult.CANCELED:
                self.get_logger().warn(f'Navigation to P{i} was canceled')
            elif result == NavigationResult.FAILED:
                self.get_logger().error(f'Failed to reach P{i}')

            time.sleep(2)  # Small delay before moving to next pose

        self.get_logger().info('Navigation task completed.')

    def create_goal_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        # Set orientation using yaw (in radians)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose


def main(args=None):
    rclpy.init(args=args)
    navigator = EBotNavigator()

    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
