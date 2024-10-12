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
import random


# ROS 2 Node class
class TestCommand(Node):
    def __init__(self):
        super().__init__('test_command_node')

        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
       
        self.get_logger().info("Node initiated")

        self.timer = self.create_timer(0.3, self.publish_commands)

    
            

    def publish_commands(self):

        control = TwistStamped()
        control.header.stamp = self.get_clock().now().to_msg()
                   
        control.twist.linear.x = random.uniform(0.0, 1.0)
        control.twist.angular.z = random.uniform(-1.0, 1.0)
        print(control.twist.linear.x , control.twist.angular.z )
        self.publisher_.publish(control)
    

def main(args=None):
    rclpy.init(args=args)
    test_command_node = TestCommand()

    try:
        while rclpy.ok():
            rclpy.spin_once(test_command_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_command_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()