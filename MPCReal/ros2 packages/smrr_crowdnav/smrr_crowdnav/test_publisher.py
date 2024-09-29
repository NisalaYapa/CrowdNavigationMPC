#!/usr/bin/env python3.8

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from smrr_interfaces.msg import Entities
import random
from time import sleep

# Publisher Node
class TestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_publisher_node')

        # Publishers for the topics
        self.human_position_pub = self.create_publisher(Entities, 'human_position', 10)
        self.human_velocity_pub = self.create_publisher(Entities, 'human_velocity', 10)
        self.human_goal_pub = self.create_publisher(Entities, 'human_goal', 10)
        self.robot_state_pub = self.create_publisher(Float32MultiArray, 'robot_state', 10)

        # Timer to publish sample data every second
        self.timer = self.create_timer(1.0, self.publish_sample_data)

        self.get_logger().info("Test Publisher Node has been started")

    def publish_sample_data(self):
        # Generate random data for human agents
        num_agents = 3
        human_pos_msg = Entities()
        human_pos_msg.count = num_agents
        human_pos_msg.x = [random.uniform(-5, 5) for _ in range(num_agents)]
        human_pos_msg.y = [random.uniform(-5, 5) for _ in range(num_agents)]

        human_vel_msg = Entities()
        human_vel_msg.count = num_agents
        human_vel_msg.x = [random.uniform(-1, 1) for _ in range(num_agents)]
        human_vel_msg.y = [random.uniform(-1, 1) for _ in range(num_agents)]

        human_goal_msg = Entities()
        human_goal_msg.count = num_agents
        human_goal_msg.x = [random.uniform(0, 10) for _ in range(num_agents)]
        human_goal_msg.y = [random.uniform(0, 10) for _ in range(num_agents)]

        # Publish human states
        self.human_position_pub.publish(human_pos_msg)
        self.human_velocity_pub.publish(human_vel_msg)
        self.human_goal_pub.publish(human_goal_msg)

        # Generate and publish random robot state
        robot_state_msg = Float32MultiArray()
        robot_state_msg.data = [
            random.uniform(-5, 5),   # px
            random.uniform(-5, 5),   # py
            random.uniform(-1, 1),   # vx
            random.uniform(-1, 1),   # vy
            random.uniform(0, 10),   # goalx
            random.uniform(0, 10),   # goaly
            random.uniform(-3.14, 3.14),  # theta
            random.uniform(-1, 1)    # omega
        ]
        self.robot_state_pub.publish(robot_state_msg)

        # Log the published data
        self.get_logger().info(f"Published human position: {human_pos_msg.x}, {human_pos_msg.y}")
        self.get_logger().info(f"Published human velocity: {human_vel_msg.x}, {human_vel_msg.y}")
        self.get_logger().info(f"Published human goal: {human_goal_msg.x}, {human_goal_msg.y}")
        self.get_logger().info(f"Published robot state: {robot_state_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    test_publisher_node = TestPublisherNode()

    try:
        rclpy.spin(test_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
