FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    cmake \
    build-essential \
    nano \
    git \
    iproute2 \
    usbutils \
    wget \
    curl \
    libserial-dev \
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
    ros-humble-ament-cmake \
    ros-humble-depthai-ros \
    ros-humble-imu-filter-madgwick \
    && rm -rf /var/lib/apt/lists/*

# Create colcon workspace
WORKDIR /root/colcon_ws/src
RUN git clone --recurse-submodules --branch final_clean https://github.com/DJacquemont/BlockBuster.git .

# Build the workspace
WORKDIR /root/colcon_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install"

# Build the serial library
WORKDIR /root
RUN git clone https://github.com/joshnewans/serial.git
WORKDIR /root/serial
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; make" && \
    make install

# .bashrc entries
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo 'cd() { builtin cd "$@" && ls; }' >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> /root/.bashrc

WORKDIR /root
COPY robot_start.sh /root
RUN chmod +x /root/robot_start.sh

# Set up the entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]