#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys
import select
import termios
import tty

# Instruction message for the user
instructions = """
--------------------------------------------------
Unitree Go2 Robot Keyboard Teleoperation Node
--------------------------------------------------
Moving around:
        w
   a    s    d
        x

q / e : Rotate left / right (Yaw)
r / f : Increase / decrease body height (z-offset)
t / g : Tilt nose up / down (Pitch compensation)

space : Emergency stop & reset all velocities

CTRL-C to quit
--------------------------------------------------
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publish Twist commands for velocity
        self.vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publish auxiliary control commands (height, pitch offset) for gait tuning
        self.aux_pub_ = self.create_publisher(Float64MultiArray, '/teleop_aux', 10)
        
        # Current states
        self.linear_x_ = 0.0
        self.linear_y_ = 0.0
        self.angular_z_ = 0.0
        self.body_height_ = 0.0      # Offset from default standing height
        self.body_pitch_ = 0.0       # Pitch tilt offset
        
        # Increments
        self.speed_step_ = 0.05
        self.yaw_step_ = 0.1
        self.height_step_ = 0.01
        self.pitch_step_ = 0.02
        
        # Save terminal settings for non-blocking read
        self.settings_ = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Keyboard Teleop Node Initialized. Press keys to control.')
        print(instructions)
        self.print_status()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)
        return key

    def print_status(self):
        # Print current values on a single line
        sys.stdout.write(
            f"\rSpeed: X={self.linear_x_:.2f} m/s, Y={self.linear_y_:.2f} m/s | "
            f"Yaw={self.angular_z_:.2f} rad/s | Height Offset={self.body_height_:.2f} m | "
            f"Pitch Offset={self.body_pitch_:.2f} rad      "
        )
        sys.stdout.flush()

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if not key:
                    continue
                
                # Check for exit
                if key == '\x03': # CTRL-C
                    break
                
                # Speed controls
                elif key == 'w':
                    self.linear_x_ += self.speed_step_
                elif key == 's':
                    self.linear_x_ -= self.speed_step_
                elif key == 'a':
                    self.linear_y_ += self.speed_step_
                elif key == 'd':
                    self.linear_y_ -= self.speed_step_
                elif key == 'q':
                    self.angular_z_ += self.yaw_step_
                elif key == 'e':
                    self.angular_z_ -= self.yaw_step_
                elif key == 'x':
                    self.linear_x_ = 0.0
                    self.linear_y_ = 0.0
                    self.angular_z_ = 0.0
                
                # Auxiliary height and tilt controls
                elif key == 'r':
                    self.body_height_ = min(0.1, self.body_height_ + self.height_step_)
                elif key == 'f':
                    self.body_height_ = max(-0.15, self.body_height_ - self.height_step_)
                elif key == 't':
                    self.body_pitch_ = min(0.3, self.body_pitch_ + self.pitch_step_)
                elif key == 'g':
                    self.body_pitch_ = max(-0.3, self.body_pitch_ - self.pitch_step_)
                
                # Emergency Stop / Reset
                elif key == ' ':
                    self.linear_x_ = 0.0
                    self.linear_y_ = 0.0
                    self.angular_z_ = 0.0
                    self.body_height_ = 0.0
                    self.body_pitch_ = 0.0
                
                # Publish the velocity Twist message
                twist = Twist()
                twist.linear.x = self.linear_x_
                twist.linear.y = self.linear_y_
                twist.angular.z = self.angular_z_
                self.vel_pub_.publish(twist)
                
                # Publish auxiliary array [height_offset, pitch_offset]
                aux_msg = Float64MultiArray()
                aux_msg.data = [self.body_height_, self.body_pitch_]
                self.aux_pub_.publish(aux_msg)
                
                self.print_status()
                
        except Exception as e:
            self.get_logger().error(f"Error in keyboard read: {e}")
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
