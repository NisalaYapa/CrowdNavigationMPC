#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from smrr_interfaces.action import NavigateToGoal
import threading
import time

class NavigateToGoalClient(Node):
    def __init__(self):
        super().__init__('navigate_to_goal_client')

        # Create an action client for 'NavigateToGoal'
        self._action_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')

        # Store the goal handle for future use (cancellation, etc.)
        self.goal_handle = None

    def send_goal(self, x, y):
        """Send a goal to the action server."""
        goal_msg = NavigateToGoal.Goal()
        goal_msg.goal_x = x
        goal_msg.goal_y = y

        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Send the goal and set up callbacks for feedback and goal response
        self.get_logger().info(f'Sending goal to coordinates: x={x}, y={y}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        #self.timer_ = self.create_timer(10.0, self.cancel_goal)

    def goal_response_callback(self, future):
        """Handle the response from the action server after sending the goal."""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted by server.')
        # Wait for the result once the goal is processed by the server
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Distance to goal = {feedback.distance_to_goal}')

    def get_result_callback(self, future):
        """Handle the result after goal execution is complete."""
        result = future.result().result

        if result.success:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().info('Failed to reach the goal.')

        # Reset the goal handle after processing the result
        self.goal_handle = None

    def cancel_goal(self):
        """Cancel the current goal if it is active."""
        if self.goal_handle and self.goal_handle.accepted:
            self.get_logger().info("Cancelling the current goal...")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

            # Cancel the timer after sending the cancel request
            self.timer_.cancel()
        else:
            self.get_logger().info("No active goal to cancel.")

    def cancel_done_callback(self, future):
        """Handle the result of the goal cancellation request."""
        cancel_response = future.result()

        if cancel_response.goals_canceling:
            self.get_logger().info("Goal cancellation accepted.")
        else:
            self.get_logger().info("Goal cancellation rejected.")


def main(args=None):
    """Main function to run the action client."""
    rclpy.init(args=args)

    # Create the action client node
    action_client = NavigateToGoalClient()
    
    # Sends goal and waits until it’s completed
    x = float(input("Enter goal x coordinate: "))
    y = float(input("Enter goal y coordinate: "))
    action_client.send_goal(x, y)

    
    #action_client.send_goal(5.0, 5.0)

    rclpy.spin(action_client)


    # # Main command loop to interact with the user
    # try:
    #     while rclpy.ok():
    #         command = input("Enter 'g' to send goal, 'c' to cancel, or 'q' to quit: ")

    #         if command == 'g':
    #             x = float(input("Enter goal x coordinate: "))
    #             y = float(input("Enter goal y coordinate: "))
    #             action_client.send_goal(x, y)
    #         elif command == 'c':
    #             action_client.cancel_goal()
    #         elif command == 'q':
    #             break

    #         # Spin once to process feedback and callbacks
    #         rclpy.spin_once(action_client, timeout_sec=0.1)  # Short timeout to allow user input

    # finally:
    #     action_client.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()
