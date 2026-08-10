# Quadruped Gait & Balance Control Pipeline
**100-Hour Challenge: Go2-Style Quadruped in Gazebo**

This repository contains a modular control architecture for a quadruped robot, developed as part of a 100-hour robotics challenge. The pipeline successfully bridges high-level keyboard commands to low-level motor torques, resulting in stable, coordinated trot walking in a simulated Gazebo environment.

## 🧠 System Architecture

The pipeline is split into three independent nodes that feed into the simulator's ros2_control interface:

### 1. Gait Generator (`gait_generator.py`)
Responsible for open-loop leg trajectory generation.
* **Trot Gait:** Utilizes a two-phase trot pattern (duty factor = 0.5) keeping diagonal leg pairs planted.
* **Kinematics:** Employs a closed-form algebraic Inverse Kinematics (IK) solver to translate 3D Cartesian target positions into 12 joint angles instantly.
* **Phase Broadcasting:** Broadcasts a `/leg_stance_state` mask, allowing downstream nodes to know exactly which feet are planted and which are swinging.

### 2. IMU Balance Controller (`imu_balance_controller.py`)
Responsible for active kinematic stabilization.
* **Filter:** Applies an Exponential Moving Average (EMA, alpha=0.8) to the raw quaternion IMU data to filter out high-frequency shockwaves from rigid-body footfalls.
* **Kinematic Override:** Uses a PD loop to adjust the $Z$-height of each foot based on the robot's roll and pitch, actively keeping the chassis level over uneven terrain.

### 3. Stance Force Controller (`stance_force_controller.py`)
Responsible for dynamic weight distribution and compliance (Virtual Spring-Damper).
* **Base Load:** Assigns a base $F_z$ of -40 N to all planted feet to support the robot's standing weight.
* **PD Posture Correction:** Acts as a virtual spring-damper by modifying the Cartesian push force based on real-time IMU tilt and angular velocity.
* **Jacobian Transpose:** Calculates the analytical Jacobian matrix ($J$) in real-time. Maps desired foot forces ($F$) to joint torques ($\tau$) using the virtual-work approach ($\tau = J^T \cdot F$), intentionally avoiding the Jacobian Inverse to prevent numerical singularities at full leg extension.

## 🔄 Node Graph & Data Flow
* `/cmd_vel` & `/teleop_aux` $\rightarrow$ `gait_generator`
* `gait_generator` $\rightarrow$ `/gait_joint_commands` & `/leg_stance_state`
* `/trunk_imu` $\rightarrow$ `imu_balance_controller` & `stance_force_controller`
* Controller Outputs $\rightarrow$ `/joint_commands` & `/joint_torques` $\rightarrow$ Simulator

## 🚀 Known Limitations & Future Work
Given the 100-hour constraint, the following shortcuts were taken and mark the roadmap for future development:
1. **Friction Forces:** The Stance Force controller currently ignores $F_x$ and $F_y$. Adding lateral force calculation is required to respect the friction cone and survive lateral shoves.
2. **Whole-Body Control:** The balance and force nodes currently operate independently (stacked architecture). Future work involves fusing them into a unified Whole-Body Controller (WBC) for true CoM tracking.
3. **Open-Loop Gait:** The gait generator relies on a time-based clock without ground-contact feedback.

## 🛠️ Usage
1. Ensure ROS 2 and Gazebo are sourced.
2. Launch the robot's hardware/simulation bridge.
3. Run the nodes in standard ROS 2 fashion:
```bash
ros2 run <package_name> gait_generator
ros2 run <package_name> imu_balance_controller
ros2 run <package_name> stance_force_controller
