#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.timer_ = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.time_ = 0.0
        self.get_logger().info('Joint Command Publisher Example Node initialized. Publishing squat motions to /joint_commands.')

    def timer_callback(self):
        # We will keep hip joints constant and oscillate thigh/calf joints to create a squatting motion
        # Default standing: hip=-0.1/0.1, thigh=0.8, calf=-1.5
        
        # Sine wave for joint oscillation
        amplitude = 0.3
        frequency = 1.0  # 1 Hz
        offset = amplitude * math.sin(2 * math.pi * frequency * self.time_)
        
        thigh_val = 0.8 + offset
        calf_val = -1.5 - offset
        
        msg = Float64MultiArray()
        msg.data = [
            -0.1, thigh_val, calf_val,  # LF
             0.1, thigh_val, calf_val,  # RF
            -0.1, thigh_val, calf_val,  # LR
             0.1, thigh_val, calf_val   # RR
        ]
        
        self.publisher_.publish(msg)
        self.time_ += 0.02

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
