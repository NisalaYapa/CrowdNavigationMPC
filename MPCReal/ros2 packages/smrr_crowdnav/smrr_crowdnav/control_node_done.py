#!/usr/bin/env python3.8

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import casadi as cs
import numpy as np
from smrr_interfaces.msg import Entities
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from time import sleep
from .NewMPCReal import NewMPCReal

# Define SelfState class
class SelfState:
    def __init__(self, px, py, vx, vy, theta, omega, gx=10, gy=10, radius=0.5, v_pref=1.0):
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
    def __init__(self, px, py, vx, vy, gx, gy, radius=0.3, v_pref=1.0):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
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

        # Initialize MPC
        self.mpc = NewMPCReal()

        # Initialize state variables
        self.self_state = None
        self.human_states = []

        # Create subscribers for the custom messages
        self.create_subscription(Entities, 'human_position', self.human_position_callback, 10)
        self.create_subscription(Entities, 'human_velocity', self.human_velocity_callback, 10)
        self.create_subscription(Entities, 'human_goal', self.human_goal_callback, 10)
        self.create_subscription(Odometry, 'robot_position', self.robot_position_callback, 10)
        self.create_subscription(Twist, 'robot_velocity', self.robot_velocity_callback, 10)
        #self.create_subscription(Float32MultiArray, 'robot_goal', self.robot_goal_callback, 10)

        # Publisher to send control commands (v, omega)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_commands', 10)

        self.get_logger().info("Node initiated")

    def human_position_callback(self, msg):
        # Update human states with position data
        self.human_states = []
        for i in range(msg.count):
            self.human_states.append(HumanState(px=msg.x[i], py=msg.y[i], vx=0.0, vy=0.0, gx=0.0, gy=0.0))

    def human_velocity_callback(self, msg):
        # Update human states with velocity data
        for i in range(msg.count):
            self.human_states[i].vx = msg.x[i]
            self.human_states[i].vy = msg.y[i]

    def human_goal_callback(self, msg):
        # Update human states with goal data
        for i in range(msg.count):
            self.human_states[i].gx = msg.x[i]
            self.human_states[i].gy = msg.y[i]

    def robot_position_callback(self, msg):
    # Extract the orientation quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        # Update the self_state with the position and orientation data
        self.self_state = SelfState(px=orientation_list[0], py=orientation_list[1],vx=0.0, vy=0.0, theta=yaw, omega=0.0)

    def robot_velocity_callback(self, msg):
            velocity = msg.linear.x
            self.self_state.vx = velocity*np.cos(self.self_state.theta)
            self.self_state.vy = velocity*np.sin(self.self_state.theta)
            self.publish_commands()

    #def robot_goal_callback(self, msg):            
            #self.self_state.gx = msg.data[0]
            #self.self_state.gy = msg.data[1]
            

    def publish_commands(self):
        # Predict and publish control commands
        if self.self_state and self.human_states:
            env_state = EnvState(self.self_state, self.human_states)
            action = self.mpc.predict(env_state)

            # Publish the control action (velocity, angular velocity)
            action_msg = Float32MultiArray()
            action_msg.data = [float(action[0]), float(action[1])]  # Assuming action is a tuple (v, omega)
            self.publisher_.publish(action_msg)

            self.get_logger().info(f"Action taken: {action}")

def main(args=None):
    rclpy.init(args=args)
    crowdnav_mpc_node = CrowdNavMPCNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(crowdnav_mpc_node)
    except KeyboardInterrupt:
        pass
    finally:
        crowdnav_mpc_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
