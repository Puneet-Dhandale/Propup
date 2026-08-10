import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import math

class GaitGenerator(Node):
    def __init__(self):
        super().__init__('gait_generator')
        
        self.joint_pub = self.create_publisher(Float64MultiArray, '/gait_joint_commands', 10)

        # NEW: Broadcasts which legs are in stance (1.0) vs swing (0.0), order [LF, RF, LR, RR]
        # so stance_force_controller can avoid commanding ground-reaction force on a leg
        # that is currently mid-air.
        self.phase_pub = self.create_publisher(Float64MultiArray, '/leg_stance_state', 10)

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)
        
        # NEW: Listen to the auxiliary keyboard commands (Pitch and Height)
        self.aux_sub = self.create_subscription(Float64MultiArray, '/teleop_aux', self.aux_callback, 10)
        
        self.timer_period = 0.02 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.time_elapsed = 0.0
        self.target_x_vel = 0.0
        self.target_y_vel = 0.0
        self.target_yaw_vel = 0.0
        
        self.body_height_offset = 0.0
        self.body_pitch_offset = 0.0
        
        self.l2 = 0.213
        self.l3 = 0.213
        self.d = 0.0955
        self.stand_height = 0.28
        self.step_clearance = 0.08
        self.gait_freq = 2.0
        self.duty_factor = 0.5

        self.get_logger().info("Gait Generator Active! Listening to velocities and Aux offsets.")

    def velocity_callback(self, msg):
        self.target_x_vel = msg.linear.x
        self.target_y_vel = msg.linear.y 
        self.target_yaw_vel = msg.angular.z

    def aux_callback(self, msg):
        """Captures height (r/f) and pitch (t/g) from the keyboard."""
        if len(msg.data) >= 2:
            self.body_height_offset = msg.data[0]
            self.body_pitch_offset = msg.data[1]

    def inverse_kinematics(self, x, y, z, is_left_leg):
        D = self.d if is_left_leg else -self.d
        L = math.sqrt(max(y*y + z*z, D*D + 1e-6))
        theta1 = math.atan2(z, y) + math.acos(D / L)
        
        dist = math.sqrt(max(x*x + y*y + z*z - D*D, 1e-6))
        cos_theta3 = (dist*dist - self.l2*self.l2 - self.l3*self.l3) / (2*self.l2*self.l3)
        cos_theta3 = max(-1.0, min(1.0, cos_theta3))
        theta3 = -math.acos(cos_theta3)
        
        z_leg = -math.sqrt(max(y*y + z*z - D*D, 1e-6))
        theta2 = (math.atan2(x, -z_leg) - math.atan2(self.l3 * math.sin(theta3), self.l2 + self.l3 * math.cos(theta3)))
        return [theta1, theta2, theta3]

    def get_local_phase(self, global_phase, is_left_leg, is_front_leg):
        """Same offset logic used inside get_trot_foot_target, factored out so we
        can also use it just to publish stance/swing state without recomputing
        the whole foot trajectory."""
        offset = 0.0 if ((is_left_leg and is_front_leg) or (not is_left_leg and not is_front_leg)) else 0.5
        return (global_phase + offset) % 1.0

    def get_trot_foot_target(self, global_phase, is_left_leg, is_front_leg):
        local_phase = self.get_local_phase(global_phase, is_left_leg, is_front_leg)
        
        step_length_x = self.target_x_vel / self.gait_freq
        step_length_y = self.target_y_vel / self.gait_freq
        
        # AMPLIFIED Yaw logic to break physical friction and turn the robot
        if is_left_leg:
            step_length_x -= (self.target_yaw_vel * 0.4)
        else:
            step_length_x += (self.target_yaw_vel * 0.4)

        x = 0.0
        y = self.d if is_left_leg else -self.d
        
        # Apply Base Height offset
        z_base = -(self.stand_height + self.body_height_offset)
        
        # Apply Kinematic Pitch offset
        if is_front_leg:
            z_base += self.body_pitch_offset
        else:
            z_base -= self.body_pitch_offset
        
        if abs(self.target_x_vel) < 0.01 and abs(self.target_y_vel) < 0.01 and abs(self.target_yaw_vel) < 0.01:
            return x, y, z_base
            
        if local_phase < self.duty_factor:
            phi_s = local_phase / self.duty_factor
            x = (step_length_x / 2.0) - (step_length_x * phi_s)
            y += (step_length_y / 2.0) - (step_length_y * phi_s)
            z = z_base
        else:
            phi_w = (local_phase - self.duty_factor) / (1.0 - self.duty_factor)
            x = -(step_length_x / 2.0) + (step_length_x * phi_w)
            y += -(step_length_y / 2.0) + (step_length_y * phi_w)
            z = z_base + (self.step_clearance * math.sin(math.pi * phi_w))

        return x, y, z

    def timer_callback(self):
        self.time_elapsed += self.timer_period
        global_phase = (self.time_elapsed * self.gait_freq) % 1.0
        
        x_fl, y_fl, z_fl = self.get_trot_foot_target(global_phase, True, True)
        x_fr, y_fr, z_fr = self.get_trot_foot_target(global_phase, False, True)
        x_rl, y_rl, z_rl = self.get_trot_foot_target(global_phase, True, False)
        x_rr, y_rr, z_rr = self.get_trot_foot_target(global_phase, False, False)
        
        fl_joints = self.inverse_kinematics(x_fl, y_fl, z_fl, True)
        fr_joints = self.inverse_kinematics(x_fr, y_fr, z_fr, False)
        rl_joints = self.inverse_kinematics(x_rl, y_rl, z_rl, True)
        rr_joints = self.inverse_kinematics(x_rr, y_rr, z_rr, False)
        
        msg = Float64MultiArray()
        msg.data = fl_joints + fr_joints + rl_joints + rr_joints
        self.joint_pub.publish(msg)

        # Publish stance (1.0) / swing (0.0) state per leg, order [LF, RF, LR, RR]
        # -- must match the leg ordering used in stance_force_controller.leg_mirrors_
        stance_state = []
        for is_left, is_front in [(True, True), (False, True), (True, False), (False, False)]:
            lp = self.get_local_phase(global_phase, is_left, is_front)
            # A leg standing still (no commanded velocity) is always in stance --
            # get_trot_foot_target() returns early with z_base in that case, so
            # mirror that same "standing still" condition here.
            standing_still = (abs(self.target_x_vel) < 0.01 and
                               abs(self.target_y_vel) < 0.01 and
                               abs(self.target_yaw_vel) < 0.01)
            is_stance = standing_still or (lp < self.duty_factor)
            stance_state.append(1.0 if is_stance else 0.0)

        phase_msg = Float64MultiArray()
        phase_msg.data = stance_state
        self.phase_pub.publish(phase_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaitGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
