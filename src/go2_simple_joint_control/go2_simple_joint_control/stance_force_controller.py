#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
import numpy as np

class StanceForceController(Node):
    def __init__(self):
        super().__init__('stance_force_controller')
        
        self.joint_state_sub_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.imu_sub_ = self.create_subscription(Imu, '/trunk_imu', self.imu_callback, 10)

        # leg stance mask from gait generator [LF, RF, LR, RR]
        # defaults to all 1.0 (stance) so the bot doesn't immediately collapse on spawn
        self.stance_state_sub_ = self.create_subscription(Float64MultiArray, '/leg_stance_state', self.stance_state_callback, 10)
        self.stance_state_ = [1.0, 1.0, 1.0, 1.0]
        
        self.torque_pub_ = self.create_publisher(Float64MultiArray, '/joint_torques', 10)
        
        self.l2_ = 0.213
        self.l3_ = 0.213
        self.d_ = 0.0955
        self.leg_mirrors_ = [1.0, -1.0, 1.0, -1.0] # LF, RF, LR, RR
        
        self.joint_positions_ = np.zeros(12)
        self.joint_names_ = [
            'LF_hip_joint', 'LF_upper_leg_joint', 'LF_lower_leg_joint',
            'RF_hip_joint', 'RF_upper_leg_joint', 'RF_lower_leg_joint',
            'LR_hip_joint', 'LR_upper_leg_joint', 'LR_lower_leg_joint',
            'RR_hip_joint', 'RR_upper_leg_joint', 'RR_lower_leg_joint'
        ]
        
        # posture state (PD)
        self.roll_ = 0.0
        self.pitch_ = 0.0
        self.roll_vel_ = 0.0
        self.pitch_vel_ = 0.0
        
        self.timer_ = self.create_timer(0.02, self.timer_callback) # 50Hz timer
        self.get_logger().info('FAST Stance Force Controller Active! Ready to hold position.')

    def imu_callback(self, msg):
        # convert quat to euler so we can actually read the tilt
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll_ = np.arctan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self.pitch_ = np.sign(sinp) * np.pi / 2.0
        else:
            self.pitch_ = np.arcsin(sinp)

        # grab angular velocities for the D term
        self.roll_vel_ = msg.angular_velocity.x
        self.pitch_vel_ = msg.angular_velocity.y

    def stance_state_callback(self, msg):
        if len(msg.data) == 4:
            self.stance_state_ = list(msg.data)

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names_:
                idx = self.joint_names_.index(name)
                self.joint_positions_[idx] = pos

    def compute_jacobian(self, leg_index, q1, q2, q3):
        # analytical jacobian matrix for a given leg
        J = np.zeros((3, 3))
        m = self.leg_mirrors_[leg_index]

        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)

        J[0, 0] = 0.0
        J[0, 1] = -self.l2_ * c2 - self.l3_ * c23
        J[0, 2] = -self.l3_ * c23
        
        J[1, 0] = -m * self.d_ * s1 + (self.l2_ * c2 + self.l3_ * c23) * c1
        J[1, 1] = (-self.l2_ * s2 - self.l3_ * s23) * s1
        J[1, 2] = -self.l3_ * s23 * s1
        
        J[2, 0] = m * self.d_ * c1 + (self.l2_ * c2 + self.l3_ * c23) * s1
        J[2, 1] = (self.l2_ * s2 + self.l3_ * s23) * c1
        J[2, 2] = self.l3_ * s23 * c1

        return J

    def timer_callback(self):
        target_torques = np.zeros(12)
        
        base_fz = -40.0 # base upward push to support standing weight
        
        # PD Gains (tuned by hand)
        # k_roll = 20.0  # barely moved the numbers next to -40N
        k_roll = 50.0    # much more stable
        kd_roll = 2.0    
        k_pitch = 50.0
        kd_pitch = 2.0   
        
        roll_correction = (k_roll * self.roll_) + (kd_roll * self.roll_vel_)
        pitch_correction = (k_pitch * self.pitch_) + (kd_pitch * self.pitch_vel_)
        
        for i in range(4):
            idx = i * 3

            # ignore legs in the air so they don't drag
            if self.stance_state_[i] < 0.5:
                continue

            # target virtual force [Fx, Fy, Fz]
            # TODO: add Fx and Fy calculations to handle lateral shoves (friction cone limitation)
            F = np.array([0.0, 0.0, base_fz])
            
            side = self.leg_mirrors_[i]  
            is_front = i in (0, 1)       
            
            F[2] += side * roll_correction
            F[2] -= (1.0 if is_front else -1.0) * pitch_correction
                
            q1, q2, q3 = self.joint_positions_[idx:idx+3]
            J = self.compute_jacobian(i, q1, q2, q3)
            
            # FIXME: This hits a singularity if the leg straightens out. 
            # Using Jacobian Transpose instead of Inverse to prevent gazebo physics engine from crashing.
            tau = J.T.dot(F)
            
            target_torques[idx:idx+3] = tau
        
        # safety clamp to prevent bad IMU reads from asking for infinite torque
        target_torques = np.clip(target_torques, -35.0, 35.0)

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