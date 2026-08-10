# 🐕 Unitree Go2 Simulation & Control Workspace

A URDF-based, spawnable Unitree Go2 quadruped for ROS 2 + Gazebo. This workspace comes with a working simulation and teleop, plus three control nodes left empty for you to implement: gait generation, IMU balancing, and stance force control.

---

## 1. Setup

### 1.1 Git Clone (Pull Code)
To get started, clone the repository to your local workspace:
```bash
mkdir -p ~/Robodog
cd ~/Robodog
git clone https://github.com/Team-Proboticists/quadruped-control-assignment.git
```

### 1.2 Install Dependencies
Ensure you have ROS 2 Humble installed on Ubuntu 22.04. Install the required simulation, controllers, and description libraries:
```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-ros-gz ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
```

### 1.3 Compile the Workspace
```bash
cd ~/Robodog/quadruped-control-assignment
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 1.4 Launch the Simulation
```bash
cd ~/Robodog/quadruped-control-assignment
source setup.sh
ros2 launch go2_simple_joint_control simple_spawn.launch.py
```

> [!IMPORTANT]
> **Sourcing Environment:** You MUST run `source setup.sh` (or `source install/setup.bash`) in **every new terminal** you open. If you see errors like `Package not found` or `Command not found`, it is because that terminal has not been sourced!

---

## 2. Workspace Structure

```
quadruped-control-assignment/src/go2_simple_joint_control/go2_simple_joint_control/
├── simple_joint_controller.py     # [READY] Forwards /joint_commands to the simulator
├── joint_command_publisher.py     # [READY] Example sinusoidal standing/squatting motion
├── keyboard_teleop.py             # [READY] Keyboard teleop → /cmd_vel, /teleop_aux
├── gait_generator.py              # [EMPTY] Challenge 1 — IK & Trot Gait Scheduler
├── imu_balance_controller.py      # [EMPTY] Challenge 2 — FK, IK & PD Balance Loop
└── stance_force_controller.py     # [EMPTY] Challenge 3 — Jacobian & Torque Mapping
```

---

## 3. Commanding the Joints

Once the simulation is running, you can control the 12 joint positions (radians) by publishing to `/joint_commands`.

### Option A: Publish via Command Line (One-off)
In a new terminal:
```bash
cd ~/Robodog/quadruped-control-assignment
source setup.sh
# Bends the joints to a stable standing position
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.4, -1.0, 0.0, 0.4, -1.0, 0.0, 0.4, -1.0, 0.0, 0.4, -1.0]}" -1
```
Use this first as a sanity check — if the robot stands up, your simulation and bridge are working correctly.

### Option B: Run the Sinusoidal Squatting Example
In a new terminal:
```bash
cd ~/Robodog/quadruped-control-assignment
source setup.sh
ros2 run go2_simple_joint_control joint_command_publisher
```

### Option C: Walking & Balance Control (Student Nodes)

* **Keyboard Teleop:**
  In a new terminal:
  ```bash
  cd ~/Robodog/quadruped-control-assignment
  source setup.sh
  ros2 run go2_simple_joint_control keyboard_teleop
  ```

* **Gait Generator** (Challenge 1 — Trot Walk & IK), once implemented:
  In a new terminal:
  ```bash
  cd ~/Robodog/quadruped-control-assignment
  source setup.sh
  ros2 run go2_simple_joint_control gait_generator
  ```

* **Active IMU Balancing** (Challenge 2), once implemented — run the gait generator remapped, then the balance controller on top of it:
  ```bash
  # Terminal 1: gait generator, output remapped so the balance controller can intercept it
  cd ~/Robodog/quadruped-control-assignment
  source setup.sh
  ros2 run go2_simple_joint_control gait_generator --ros-args --remap /joint_commands:=/gait_joint_commands

  # Terminal 2: balance controller — stabilizes gait output and publishes final /joint_commands
  cd ~/Robodog/quadruped-control-assignment
  source setup.sh
  ros2 run go2_simple_joint_control imu_balance_controller
  ```

* **Stance Force Control** (Challenge 3 — Jacobian Transpose, τ = Jᵀ F), once implemented:
  In a new terminal:
  ```bash
  cd ~/Robodog/quadruped-control-assignment
  source setup.sh
  ros2 run go2_simple_joint_control stance_force_controller
  ```

---

## 4. Troubleshooting

If a previous run didn't close cleanly, Gazebo processes can linger in the background and cause blank/white screens, port conflicts, or spawner timeouts. Clean them up with:
```bash
killall -9 ruby ign gazebo gz robot_state_publisher parameter_bridge spawner simple_joint_controller joint_command_publisher 2>/dev/null
```
Then re-launch the simulation (Section 1.4).

---

## 5. 🎓 Student Coding Challenge Guide

This workspace is pre-configured with educational templates to test your understanding of legged robotics kinematics, gait scheduling, and control theory. Complete the missing algorithms in the three nodes below. **All three are required** — implementation language is up to you (Python, C++, etc.), as long as your node integrates with the existing topics and runs correctly.

**Leg parameters** (used throughout): thigh length l₂ = 0.213 m, calf length l₃ = 0.213 m, hip offset d = 0.0955 m (+d for LHS legs, −d for RHS legs).

### Challenge 1: Inverse Kinematics & Trot Gait Scheduler
**Target file:** `gait_generator.py`

1. **Analytical Inverse Kinematics** (`inverse_kinematics`) — given a target foot coordinate (x, y, z) relative to the hip, compute joint angles θ₁, θ₂, θ₃:
   ```
   θ1 = atan2(z, y) ± acos(d / √(y² + z²))
   D  = √(x² + y² + z² − d²)
   θ3 = −acos((D² − l2² − l3²) / (2 l2 l3))
   θ2 = atan2(x, −z_leg) − atan2(l3 sin(−θ3), l2 + l3 cos(−θ3))
   ```
2. **Gait Timing Scheduler** (`timer_callback`) — write a 2-phase Trot cycle (50% duty factor). Diagonal leg pairs (LF/RR and RF/LR) swing and stand out of phase. Handle smooth foot-trajectory transitions between Swing and Stance.

### Challenge 2: Active IMU Balancing (Tilt Stabilization)
**Target file:** `imu_balance_controller.py`

1. **Forward Kinematics** (`forward_kinematics`) — compute the foot's (x, y, z) position from the current joint angles:
   ```
   x_leg = l2 sin(θ2) + l3 sin(θ2 + θ3)
   z_leg = −l2 cos(θ2) − l3 cos(θ2 + θ3)
   x = x_leg
   y = d cos(θ1) − z_leg sin(θ1)
   z = d sin(θ1) + z_leg cos(q1)
   ```
2. **PD Stance Height Adjustment** (`joint_callback`) — read roll (φ) and pitch (θ) from `/trunk_imu`. Design a PD loop for per-foot height correction:
   ```
   Δz_balance = Kp · error − Kd · angular_rate
   ```
   Adjust LHS vs. RHS leg height for roll stabilization, and Front vs. Rear leg height for pitch stabilization. Solve IK on the updated foot coordinates and publish the new joint commands.

### Challenge 3: Stance Force & Jacobian Torque Mapping (VMC) — Required
**Target file:** `stance_force_controller.py`

Gait and IMU control only decide *where* the feet go — not *how hard* they push on the ground. Without regulating that force, weight distribution across the legs is uneven, standing legs end up too stiff or too soft, and a bump or uneven surface can tip the robot or cause the joints to jerk. Ground reaction force control is what keeps body height steady, spreads load evenly across all four legs, and lets the robot absorb disturbances instead of fighting them — which is why this challenge is required, not optional.

1. **Jacobian Calculation** (`compute_jacobian`) — derive and compute the analytical 3×3 Jacobian matrix J = ∂(x, y, z) / ∂(θ₁, θ₂, θ₃) for the leg.
2. **Jacobian Transpose Mapping** (`timer_callback`) — using Virtual Model Control (VMC), compute the vertical force vector Fz needed to hold the robot at a target standing height (virtual spring-damper system). Map the virtual force F = [Fx, Fy, Fz]ᵀ to joint torques:
   ```
   τ = Jᵀ F
   ```
   Publish the resulting joint torques to `/joint_torques`.
