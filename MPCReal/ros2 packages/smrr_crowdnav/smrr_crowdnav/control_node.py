#!/usr/bin/env python3.8

import rclpy
import tf_transformations
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import casadi as cs
import numpy as np
from smrr_interfaces.msg import Entities
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from time import sleep
from .NewMPCReal import NewMPCReal
from .dwa import DynamicWindowApproach
from .include.transform import GeometricTransformations


# Define SelfState class
class SelfState:
    def __init__(self, px, py, vx, vy, theta, omega, gx=-3, gy=-3, radius=0.25, v_pref=0.5):
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
    def __init__(self, px, py, vx, vy, gx, gy, radius=0.15, v_pref=1):
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
        self.policy = NewMPCReal()
        #self.policy = DynamicWindowApproach()

        # Initialize state variables
        self.self_state = None
        self.human_states = []
        self.ready = True
        
        self.self_state = SelfState(px=0.0, py=0.0, vx=0.0, vy=0.0, theta=0.0, omega=0.0)

        # Create subscribers for the custom messages
        # Create subscribers for the custom messages
        self.create_subscription(Entities, '/laser_data_array', self.human_position_callback, 10)
        self.create_subscription(Entities, '/vel', self.human_velocity_callback, 10)
        self.create_subscription(Entities, '/goals', self.human_goal_callback, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.robot_velocity_callback, 10)

        # Publisher to send control commands (v, omega)
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
       
        self.get_logger().info("Node initiated")

        self.timer = self.create_timer(0.7, self.publish_commands)

        self.transform = GeometricTransformations(self)                   

    def human_position_callback(self, msg):
        # Update human states with position data
        self.human_states = []
        for i in range(msg.count):
            self.human_states.append(HumanState(px=msg.x[i], py=msg.y[i], vx=0.0, vy=0.0, gx=0.0, gy=0.0))
        #print(len(self.human_states))

    def human_velocity_callback(self, msg):
        # Update human states with velocity data
        for i in range(msg.count):
            try:
                self.human_states[i].vx = msg.x[i]
                self.human_states[i].vy = msg.y[i]
            except :
                pass

    def human_goal_callback(self, msg):
        # Update human states with goal data
        for i in range(msg.count):
            try:
                self.human_states[i].gx = msg.x[i]
                self.human_states[i].gy = msg.y[i]
            except :
                pass

    def robot_velocity_callback(self, msg):
            
            linear_x = msg.twist.twist.linear.x
            #print(f"linear x{linear_x}")

            transformation   = self.transform.get_transform('map', 'base_link') 

            if transformation is None:
                self.ready = False
                self.get_logger().info("Robot not ready: No valid transformation")
                return
            else:
                self.ready = True
                self.get_logger().info("Robot ready: Transformation found")

            quaternion       = (transformation.rotation.x, transformation.rotation.y, transformation.rotation.z, transformation.rotation.w)
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

            
            self.self_state.px    = transformation.translation.x
            self.self_state.py    = transformation.translation.y
            self.self_state.theta = yaw
            self.self_state.vx    = linear_x*np.cos(self.self_state.theta)
            self.self_state.vy    = linear_x*np.sin(self.self_state.theta)
            self.self_state.position = (self.self_state.px, self.self_state.py)
            self.self_state.omega = msg.twist.twist.angular.z
       
            

    def publish_commands(self):
        # Predict and publish control commands
        if self.self_state and self.human_states and self.ready:
            env_state = EnvState(self.self_state, self.human_states)
            print(f"robot state: {env_state.self_state.px, env_state.self_state.py, env_state.self_state.theta}")
            print(f"human state: {env_state.human_states[0].px, env_state.human_states[0].py}")
            
            action = self.policy.predict(env_state)


            control = TwistStamped()
            control.header.stamp = self.get_clock().now().to_msg()
             

            dist_to_goal = np.linalg.norm(np.array(self.self_state.position) - np.array(self.self_state.goal_position))
            print(f" current position: {self.self_state.position} goal position: {self.self_state.goal_position} distance to goal: {dist_to_goal}")
            
            if (dist_to_goal >= 1):            
                control.twist.linear.x = float(action[0])
                control.twist.angular.z = float(action[1])
                #control.twist.linear.x = 0.0
                #control.twist.angular.z = 0.2
                self.publisher_.publish(control)
                print(f"Action taken Solved: {control}")

            else:
                control.twist.linear.x = 0.0
                control.twist.angular.z = 0.0
                self.publisher_.publish(control)
                print(f"Action taken: {control}")

            #self.get_logger().info(f"Action taken: {control}")

    

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