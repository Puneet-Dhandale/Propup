#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
import numpy as np

class StanceForceController(Node):
    def __init__(self):
        super().__init__('stance_force_controller')
        
        # Subscriptions
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.imu_sub_ = self.create_subscription(
            Imu,
            '/trunk_imu',
            self.imu_callback,
            10
        )

        # NEW: which legs are currently in stance (1.0) vs swing (0.0), order [LF, RF, LR, RR].
        # Published by gait_generator. Default to "all in stance" so the robot doesn't
        # go limp before the first message arrives.
        self.stance_state_sub_ = self.create_subscription(
            Float64MultiArray,
            '/leg_stance_state',
            self.stance_state_callback,
            10
        )
        self.stance_state_ = [1.0, 1.0, 1.0, 1.0]
        
        # Publisher for target joint torques (efforts)
        self.torque_pub_ = self.create_publisher(
            Float64MultiArray,
            '/joint_torques',
            10
        )
        
        # Robot Constants
        self.l2_ = 0.213
        self.l3_ = 0.213
        self.d_ = 0.0955
        self.leg_mirrors_ = [1.0, -1.0, 1.0, -1.0] # LF, RF, LR, RR
        
        # Joint variables (LF, RF, LR, RR)
        self.joint_positions_ = np.zeros(12)
        self.joint_names_ = [
            'LF_hip_joint', 'LF_upper_leg_joint', 'LF_lower_leg_joint',
            'RF_hip_joint', 'RF_upper_leg_joint', 'RF_lower_leg_joint',
            'LR_hip_joint', 'LR_upper_leg_joint', 'LR_lower_leg_joint',
            'RR_hip_joint', 'RR_upper_leg_joint', 'RR_lower_leg_joint'
        ]
        
        self.roll_ = 0.0
        self.pitch_ = 0.0
        
        self.timer_ = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('FAST Stance Force Controller Active! Ready to hold position.')

    def imu_callback(self, msg):
        q = msg.orientation
        
        # Convert Quaternion to Roll and Pitch
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll_ = np.arctan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self.pitch_ = np.sign(sinp) * np.pi / 2.0
        else:
            self.pitch_ = np.arcsin(sinp)

    def stance_state_callback(self, msg):
        if len(msg.data) == 4:
            self.stance_state_ = list(msg.data)

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names_:
                idx = self.joint_names_.index(name)
                self.joint_positions_[idx] = pos

    def compute_jacobian(self, leg_index, q1, q2, q3):
        """
        Computes the 3x3 Analytical Jacobian Matrix J for a given leg.
        """
        J = np.zeros((3, 3))
        m = self.leg_mirrors_[leg_index]

        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)

        # Partial Derivatives of X
        J[0, 0] = 0.0
        J[0, 1] = -self.l2_ * c2 - self.l3_ * c23
        J[0, 2] = -self.l3_ * c23
        
        # Partial Derivatives of Y
        J[1, 0] = -m * self.d_ * s1 + (self.l2_ * c2 + self.l3_ * c23) * c1
        J[1, 1] = (-self.l2_ * s2 - self.l3_ * s23) * s1
        J[1, 2] = -self.l3_ * s23 * s1
        
        # Partial Derivatives of Z
        J[2, 0] = m * self.d_ * c1 + (self.l2_ * c2 + self.l3_ * c23) * s1
        J[2, 1] = (self.l2_ * s2 + self.l3_ * s23) * c1
        J[2, 2] = self.l3_ * s23 * c1

        return J

    def timer_callback(self):
        target_torques = np.zeros(12)
        
        # Base upward force required per leg to keep the robot standing
        base_fz = -40.0 
        
        # Proportional gains for posture correction
        k_roll = 50.0
        k_pitch = 50.0
        
        # Calculate corrective forces based on IMU error
        roll_correction = k_roll * self.roll_
        pitch_correction = k_pitch * self.pitch_
        
        for i in range(4):
            idx = i * 3

            # If this leg is currently swinging (foot in the air), it should not
            # be commanded any ground-reaction force -- leave its torques at 0
            # (already zero-initialized in target_torques) and skip it entirely.
            if self.stance_state_[i] < 0.5:
                continue

            # Target virtual force the foot exerts on the ground [Fx, Fy, Fz]
            F = np.array([0.0, 0.0, base_fz])
            
            side = self.leg_mirrors_[i]  # +1.0 for Left, -1.0 for Right
            is_front = i in (0, 1)       # True for Front, False for Rear
            
            # Apply Roll Correction (Left legs push harder if leaning left, etc.)
            F[2] += side * roll_correction
            
            # Apply Pitch Correction (Front legs push harder if leaning forward)
            F[2] -= (1.0 if is_front else -1.0) * pitch_correction
                
            # Get current joint angles for this specific leg
            q1 = self.joint_positions_[idx]
            q2 = self.joint_positions_[idx + 1]
            q3 = self.joint_positions_[idx + 2]
            
            # Calculate the Jacobian
            J = self.compute_jacobian(i, q1, q2, q3)
            
            # Map virtual force to joint torques: tau = J^T * F
            tau = J.T.dot(F)
            
            # Assign computed torques
            target_torques[idx]     = tau[0]  # Hip torque
            target_torques[idx + 1] = tau[1]  # Thigh torque
            target_torques[idx + 2] = tau[2]  # Calf torque
        
        msg = Float64MultiArray()
        msg.data = list(target_torques)
        self.torque_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StanceForceController()
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
