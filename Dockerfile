FROM osrf/ros:humble-desktop
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    lsb-release wget gnupg \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-xacro \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y \
    ignition-fortress \
    ros-humble-ros-gz \
    ros-humble-ign-ros2-control \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /home/ros2_ws/setup.sh ]; then source /home/ros2_ws/setup.sh; fi" >> /root/.bashrc

WORKDIR /home/ros2_ws
