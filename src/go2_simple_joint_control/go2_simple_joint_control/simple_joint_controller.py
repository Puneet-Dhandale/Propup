#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class SimpleJointControllerBridge(Node):
    """
    Acts as a bridge between the custom quadruped controllers (which output simple float arrays)
    and the standard ROS 2 ros2_control trajectory interface for Gazebo.
    """
    def __init__(self):
        super().__init__('simple_joint_controller_bridge')
        
        # --- Subscriptions ---
        # Listens for the fused output from the IMU Balance Controller.
        self.subscriber_ = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.command_callback,
            10
        )
        
        # --- Publishers ---
        # Forwards the formatted trajectory to the Gazebo hardware interface.
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            10
        )
        
        # Exact joint order mapping required by the go2_description's effort controller.
        self.joint_names_ = [
            'LF_hip_joint', 'LF_upper_leg_joint', 'LF_lower_leg_joint',
            'RF_hip_joint', 'RF_upper_leg_joint', 'RF_lower_leg_joint',
            'LR_hip_joint', 'LR_upper_leg_joint', 'LR_lower_leg_joint',
            'RR_hip_joint', 'RR_upper_leg_joint', 'RR_lower_leg_joint'
        ]
        
        # Default stable standing posture (rads).
        # Order: LF, RF, LR, RR (Hip, Thigh, Calf).
        self.default_standing_posture_ = [
            -0.1, 0.8, -1.5,  # Left Front.
             0.1, 0.8, -1.5,  # Right Front.
            -0.1, 0.8, -1.5,  # Left Rear.
             0.1, 0.8, -1.5   # Right Rear.
        ]
        self.received_student_command_ = False
        
        # --- Startup Configuration ---
        # Parameter to determine if the robot starts limp (passive) or holds a standing pose (active).
        self.declare_parameter('start_passive', True)
        self.start_passive_ = self.get_parameter('start_passive').get_parameter_value().bool_value
        
        if not self.start_passive_:
            # If active, publish the default standing posture at 10Hz until a command arrives.
            self.timer_ = self.create_timer(0.1, self.timer_callback)
            self.get_logger().info('Starting in ACTIVE mode: holding default standing posture.')
        else:
            self.timer_ = None
            self.get_logger().info('Starting in PASSIVE mode: robot is limp. Waiting for commands on /joint_commands...')
        
        self.get_logger().info('Simple Joint Control Bridge initialized.')

    def publish_posture(self, positions):
        """Wraps a flat array of 12 joint positions into a ROS 2 JointTrajectory message."""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names_
        
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        
        # Provide a 100ms execution window for the joints to reach the target smoothly.
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        
        trajectory_msg.points.append(point)
        self.publisher_.publish(trajectory_msg)

    def timer_callback(self):
        """Maintains the default standing pose if no external commands have been received yet."""
        if not self.received_student_command_:
            self.publish_posture(self.default_standing_posture_)

    def command_callback(self, msg):
        """Callback for incoming /joint_commands from the custom controllers."""
        if len(msg.data) != 12:
            self.get_logger().warn(f'Invalid joint command size! Expected 12 values, got {len(msg.data)}')
            return
            
        # Flag that active commands are flowing, overriding the startup timer.
        self.received_student_command_ = True
        self.publish_posture(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointControllerBridge()
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