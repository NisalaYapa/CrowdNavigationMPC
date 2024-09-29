#!/usr/bin/env python3.8

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import casadi as cs
import numpy as np
import logging
from .NewMPCReal import NewMPCReal
from time import sleep  # Import sleep to create a delay

# Define SelfState class
class SelfState:
    def __init__(self, px, py, vx, vy, theta, omega, gx, gy, radius, v_pref):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        self.theta = theta
        self.omega = omega
        self.gx = gx
        self.gy = gy
        self.radius = radius
        self.v_pref = v_pref
        self.position = (self.px, self.py)
        self.goal_position = (self.gx, self.gy)
        self.velocity = (self.vx, self.vy)

# Define HumanState class
class HumanState:
    def __init__(self, px, py, vx, vy, theta, omega, gx, gy, radius, v_pref):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        self.theta = theta
        self.omega = omega
        self.gx = gx
        self.gy = gy
        self.radius = radius
        self.v_pref = v_pref
        self.position = (self.px, self.py)
        self.goal_position = (self.gx, self.gy)
        self.velocity = (self.vx, self.vy)

# Define EnvState class
class EnvState:
    def __init__(self, self_state, human_states, static_obs=[]):
        self.self_state = self_state
        self.human_states = human_states
        self.static_obs = static_obs

# ROS 2 Node class
class CrowdNavMPCNode(Node):
    def __init__(self):
        super().__init__('crowdnav_mpc_node')

        # Publisher to send control commands (v, omega)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_commands', 10)

        print("Node initiated")

        # Initialize MPC
        self.mpc = NewMPCReal()

    def publish_commands(self):
        # Example environment state data (you can modify these values as needed)
        self_state = SelfState(px=0.0, py=0.0, vx=0.0, vy=0.0, theta=0.0, omega=0.0, gx=5.0, gy=5.0, radius=0.5, v_pref=1.0)
        human1 = HumanState(px=2, py=2, vx=0.1, vy=0.1, theta=0, omega=0, gx=4, gy=4, radius=0.3, v_pref=1.0)
        human2 = HumanState(px=-2, py=-2, vx=-0.1, vy=-0.1, theta=0, omega=0, gx=-4, gy=-4, radius=0.3, v_pref=1.0)
        human_states = [human1, human2]
        env_state = EnvState(self_state, human_states)

        # Predict action using MPC based on the provided environment state
        action = self.mpc.predict(env_state)

        # Publish the control action (velocity, angular velocity)
        action_msg = Float32MultiArray()
        action_msg.data = [action[0], action[1]]  # Assuming action is a tuple (v, omega)        
        self.publisher_.publish(action_msg)

        # Log information
        self.get_logger().info(f"Action taken: {action}")

def main(args=None):
    rclpy.init(args=args)
    crowdnav_mpc_node = CrowdNavMPCNode()

    try:
        # Keep the node running and publishing commands every 0.5 seconds
        while rclpy.ok():
            crowdnav_mpc_node.publish_commands()  # Publish commands directly
            crowdnav_mpc_node.publish_commands()
            rclpy.spin_once(crowdnav_mpc_node)  # Ensure callbacks and keep the node alive
            sleep(0.5)  # Publish commands every 0.5 seconds
    except KeyboardInterrupt:
        pass
    finally:
        crowdnav_mpc_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
