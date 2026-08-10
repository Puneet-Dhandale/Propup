#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import math

class ImuBalanceController(Node):
    def __init__(self):
        super().__init__('imu_balance_controller')
        self.roll_filtered_ = 0.0
        self.pitch_filtered_ = 0.0
        
        self.joint_sub_ = self.create_subscription(Float64MultiArray, '/gait_joint_commands', self.joint_callback, 10)
        self.imu_sub_ = self.create_subscription(Imu, '/trunk_imu', self.imu_callback, 10)
        self.aux_sub_ = self.create_subscription(Float64MultiArray, '/teleop_aux', self.aux_callback, 10)
        self.joint_pub_ = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        
        self.l2_ = 0.213
        self.l3_ = 0.213
        self.d_ = 0.0955
        self.leg_mirrors_ = [1.0, -1.0, 1.0, -1.0] 
        
        self.roll_ = 0.0
        self.pitch_ = 0.0
        self.roll_vel_ = 0.0
        self.pitch_vel_ = 0.0
        
        self.target_pitch_ = 0.0

        self.kp_roll_  = 0.015
        self.kd_roll_  = 0.003
        self.kp_pitch_ = 0.015
        self.kd_pitch_ = 0.003
        self.max_correction_ = 0.015
        
        self.get_logger().info('FAST IMU Balance Node Active! Preventing tip-overs.')

    def euler_from_quaternion(self, x, y, z, w):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1.0 else math.copysign(math.pi / 2.0, sinp)
        
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def imu_callback(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.roll_, self.pitch_, _ = self.euler_from_quaternion(qx, qy, qz, qw)
        
        # EMA filter. alpha = 0.8 introduces some phase lag, but a lower value
        # let the high-frequency gazebo rigid body collisions shake the bot apart.
        alpha = 0.8
        self.roll_filtered_ = (alpha * self.roll_filtered_ + (1.0 - alpha) * self.roll_)
        self.pitch_filtered_ = (alpha * self.pitch_filtered_ + (1.0 - alpha) * self.pitch_)
        
        self.roll_vel_ = msg.angular_velocity.x
        self.pitch_vel_ = msg.angular_velocity.y

    def aux_callback(self, msg):
        # stop the IMU from fighting the user's manual pitch commands
        if len(msg.data) >= 2:
            self.target_pitch_ = msg.data[1]

    def forward_kinematics(self, leg_index, theta1, theta2, theta3):
        D = self.leg_mirrors_[leg_index] * self.d_
        x_leg = self.l2_ * math.sin(theta2) + self.l3_ * math.sin(theta2 + theta3)
        z_leg = -self.l2_ * math.cos(theta2) - self.l3_ * math.cos(theta2 + theta3)
        
        x = x_leg
        y = D * math.cos(theta1) - z_leg * math.sin(theta1)
        z = D * math.sin(theta1) + z_leg * math.cos(theta1)
        return x, y, z

    def inverse_kinematics(self, leg_index, x, y, z):
        D = self.leg_mirrors_[leg_index] * self.d_
        L = math.sqrt(max(y*y + z*z, D*D + 1e-6))
        theta1 = math.atan2(z, y) + math.acos(D / L)
        
        dist = math.sqrt(max(x*x + y*y + z*z - D*D, 1e-6))
        cos_theta3 = (dist*dist - self.l2_*self.l2_ - self.l3_*self.l3_) / (2*self.l2_*self.l3_)
        cos_theta3 = max(-1.0, min(1.0, cos_theta3))
        theta3 = -math.acos(cos_theta3)
        
        z_leg = -math.sqrt(max(y*y + z*z - D*D, 1e-6))
        theta2 = (math.atan2(x, -z_leg) - math.atan2(self.l3_ * math.sin(theta3), self.l2_ + self.l3_ * math.cos(theta3)))
        return theta1, theta2, theta3

    def joint_callback(self, msg):
        if len(msg.data) != 12:
            return
            
        raw_joints = msg.data
        balanced_joints = []

        roll_error = 0.0 - self.roll_filtered_
        pitch_error = self.target_pitch_ - self.pitch_filtered_

        roll_correction = (self.kp_roll_ * roll_error) - (self.kd_roll_ * self.roll_vel_)
        pitch_correction = (self.kp_pitch_ * pitch_error) - (self.kd_pitch_ * self.pitch_vel_)

        roll_correction  = max(-self.max_correction_, min(self.max_correction_, roll_correction))
        pitch_correction = max(-self.max_correction_, min(self.max_correction_, pitch_correction))

        for i in range(4):
            theta1, theta2, theta3 = raw_joints[3*i : 3*i+3]
            x, y, z = self.forward_kinematics(i, theta1, theta2, theta3)

            side = self.leg_mirrors_[i]
            is_front = i in (0, 1)

            roll_sign = -side
            pitch_sign = 1.0 if is_front else -1.0

            dz = (roll_sign * roll_correction) + (pitch_sign * pitch_correction)
            z_target = z + dz

            # prevent asking for a Z-height longer than the physical leg
            if abs(z_target) > (self.l2_ + self.l3_):
                z_target = math.copysign(self.l2_ + self.l3_ - 1e-3, z_target)

            t1, t2, t3 = self.inverse_kinematics(i, x, y, z_target)
            balanced_joints.extend([t1, t2, t3])
            
        out_msg = Float64MultiArray()
        out_msg.data = balanced_joints
        self.joint_pub_.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuBalanceController()
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