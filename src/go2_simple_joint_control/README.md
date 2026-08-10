# Go2 Simple Joint Control (Warm-up / Kick-off Task)

This package contains a simplified template designed as a warm-up task (Part 0) for students to learn how URDF models are spawned in Gazebo and how to control joint positions directly before moving on to the full walking control loop.

---

## 1. How it works:
1. Spawns the static Unitree Go2 URDF model directly in Gazebo.
2. Loads the standard ROS 2 Control joint state and effort/trajectory controller.
3. Launches a **Bridge Node** (`simple_joint_controller`) that listens to a simple float array topic (`/joint_commands`).
4. Students can publish an array of 12 joint angles (radians) to `/joint_commands`, and the bridge automatically packages it into the standard ROS 2 trajectory message to command the legs.

### Joint Array Order (12 elements):
1. `LF_hip_joint`, `LF_upper_leg_joint`, `LF_lower_leg_joint` (Left Front Leg)
2. `RF_hip_joint`, `RF_upper_leg_joint`, `RF_lower_leg_joint` (Right Front Leg)
3. `LR_hip_joint`, `LR_upper_leg_joint`, `LR_lower_leg_joint` (Left Rear Leg)
4. `RR_hip_joint`, `RR_upper_leg_joint`, `RR_lower_leg_joint` (Right Rear Leg)

---

## 2. How to Run:

### Step 1: Launch Gazebo Sim and Spawn the Robot
In a clean, sourced terminal:
```bash
cd ~/Robodog/quadruped-control-assignment
source setup.sh
ros2 launch go2_simple_joint_control simple_spawn.launch.py
```
*(Gazebo will load the empty world, spawn the robot standing, and start the controller bridge).*

### Step 2: Command the Joints

#### Option A: Command Line (One-off message)
In a separate terminal:
```bash
cd ~/Robodog/quadruped-control-assignment
source setup.sh
# Command the robot legs to bend to a stable standing posture
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.4, -1.0, 0.0, 0.4, -1.0, 0.0, 0.4, -1.0, 0.0, 0.4, -1.0]}" -1
```

#### Option B: Run the Example Python script (Squat loop)
In a separate terminal:
```bash
cd ~/Robodog/quadruped-control-assignment
source setup.sh
# Run the sinusoidal squat trajectory node
ros2 run go2_simple_joint_control joint_command_publisher
```

---

## 3. Writing Custom Control Scripts:
Students can write their own ROS 2 publisher nodes in Python to publish 12 joint values to `/joint_commands` based on kinematics equations or trajectory schedules. See [joint_command_publisher.py](go2_simple_joint_control/joint_command_publisher.py) for a complete template.

## 3. Advanced Walking and Balance Control Nodes

We have added new control nodes as educational templates for walking, active balancing, and force control:

### A. Keyboard Teleoperation (`keyboard_teleop`)
* **File:** [keyboard_teleop.py](go2_simple_joint_control/keyboard_teleop.py)
* **What it does:** Reads keypresses from the terminal and publishes body velocity command targets (`/cmd_vel`) and tuning adjustments (`/teleop_aux`: height and pitch).
* **Controls:**
  - `w` / `s`: Forward / backward velocity
  - `a` / `d`: Lateral (left / right) velocity
  - `q` / `e`: Heading rotation (yaw) rate
  - `r` / `f`: Body height offset adjustment (up / down)
  - `t` / `g`: Body pitch tilt adjustment (tilt nose up / down)
  - `space`: Reset all commands (Emergency Stop)
* **How to run:**
  ```bash
  ros2 run go2_simple_joint_control keyboard_teleop
  ```

### B. Gait Cycle Generator (`gait_generator`)
* **File:** [gait_generator.py](go2_simple_joint_control/gait_generator.py)
* **What it does:** Subscribes to `/cmd_vel` and `/teleop_aux`. It schedules a 2-phase (Swing vs. Stance) Trot gait cycle for all 4 legs, computes foot trajectories $(x, y, z)$ relative to the hip base frame, solves the **Inverse Kinematics (IK)** for the 3-DOF leg configuration, and outputs joint angles to `/joint_commands`.
* **Students can edit:** Look for the `# STUDENT EXERCISE / GAIT SCHEDULER` comments to customize swing curves, stride lengths, or phase offsets.
* **How to run:**
  ```bash
  ros2 run go2_simple_joint_control gait_generator
  ```

### C. Active IMU Balance Controller (`imu_balance_controller`)
* **File:** [imu_balance_controller.py](go2_simple_joint_control/imu_balance_controller.py)
* **What it does:** 
  1. Subscribes to raw walking joint angles from `/gait_joint_commands` (students should run `gait_generator` remapped to `/gait_joint_commands`).
  2. Subscribes to `/trunk_imu` to read body roll, pitch, and angular velocities.
  3. Uses **Forward Kinematics (FK)** to compute the current $(x, y, z)$ positions of the feet relative to the hip.
  4. Runs a PD/PID controller to adjust foot height offsets ($\Delta z$) to stabilize body roll and pitch errors.
  5. Computes corrected joint angles using **Inverse Kinematics (IK)** and publishes to `/joint_commands`.
* **Students can edit:** Tune the PID gains `kp_roll_`, `kd_roll_`, `kp_pitch_`, `kd_pitch_` inside the python code to balance the robot.
* **How to run:**
  ```bash
  # Step 1: Run gait generator publishing to gait commands
  ros2 run go2_simple_joint_control gait_generator --ros-args --remap /joint_commands:=/gait_joint_commands
  # Step 2: Run active balancing node that intercepts and outputs to /joint_commands
  ros2 run go2_simple_joint_control imu_balance_controller
  ```

### D. Stance Force Controller (`stance_force_controller`)
* **File:** [stance_force_controller.py](go2_simple_joint_control/stance_force_controller.py)
* **What it does:** Demonstrates **Virtual Model Control (VMC)**. It calculates the analytical 3x3 **Jacobian Matrix ($J$)** for each leg, computes a virtual force vector $F = [F_x, F_y, F_z]^T$ needed on each foot to maintain height and balance, and uses the Jacobian Transpose relation ($\tau = J^T F$) to map foot forces to joint torques (efforts).
* **How to run:**
  ```bash
  ros2 run go2_simple_joint_control stance_force_controller
  ```
