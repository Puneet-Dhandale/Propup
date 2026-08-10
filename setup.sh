#!/bin/bash
# Auto-setup script for Unitree Go2 Joint Control Assignment

# Optional: Force discrete NVIDIA GPU rendering on hybrid dual-GPU laptops. 
# (Only uncomment if you experience a blank/white screen or performance issues, as it can cause OpenGL context crashes on some setups)
# export __NV_PRIME_RENDER_OFFLOAD=1
# export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Optional: Clean up Qt Quick software backend envs if QML paint failures occur
# unset QT_QUICK_BACKEND
# unset QT_XCB_GL_INTEGRATION
# unset QT_QPA_PLATFORM


# Get the absolute directory where this setup.sh script resides
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "Sourcing local workspace..."
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# Export resource path for Gazebo meshes
if ros2 pkg prefix go2_description &>/dev/null; then
    GO2_DESC_SHARE=$(ros2 pkg prefix go2_description)/share
    export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$GO2_DESC_SHARE
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$GO2_DESC_SHARE
    echo "Configured Gazebo Sim resource paths."
else
    echo "Warning: go2_description package not found. Make sure workspace is built."
fi

# Export path for custom Gazebo system plugins
if ros2 pkg prefix gz_quadruped_hardware &>/dev/null; then
    GZ_HARDWARE_LIB=$(ros2 pkg prefix gz_quadruped_hardware)/lib
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:$GZ_HARDWARE_LIB
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$GZ_HARDWARE_LIB
    echo "Configured Gazebo Sim system plugin paths."
else
    echo "Warning: gz_quadruped_hardware package not found. Make sure workspace is built."
fi

# Check for stale simulation processes
if pgrep -f -z 0 "ign|gazebo|gz|parameter_bridge|spawner" &>/dev/null; then
    echo "----------------------------------------------------------------------------------"
    echo "WARNING: Stale Gazebo or ROS processes detected in the background!"
    echo "This can cause blank/white screens or controller manager loading timeouts."
    echo "Please run the following command to clean them up:"
    echo "killall -9 ruby ign gazebo gz robot_state_publisher parameter_bridge spawner simple_joint_controller joint_command_publisher 2>/dev/null"
    echo "----------------------------------------------------------------------------------"
fi
