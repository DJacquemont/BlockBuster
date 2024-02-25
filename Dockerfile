FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    cmake \
    nano \
    git \
    iproute2 \
    usbutils \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-rviz-plugins \
    ros-humble-teleop-twist-keyboard \
    ros-humble-twist-mux \
    ros-humble-rqt \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*

# Create colcon workspace and clone the rplidar_ros2 project
WORKDIR /colcon_ws/src
RUN git clone https://github.com/babakhani/rplidar_ros2.git
COPY articubot_one articubot_one

# Add lines to source setup.bash in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /colcon_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc

# Add lines to show folder contents on cd
RUN echo 'cd() {' >> /root/.bashrc
RUN echo '    builtin cd "$@" && ls' >> /root/.bashrc
RUN echo '}' >> /root/.bashrc

# Build the workspace
WORKDIR /colcon_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install"

# Set up the entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
