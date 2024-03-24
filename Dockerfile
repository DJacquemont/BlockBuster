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
    && rm -rf /var/lib/apt/lists/*

# Create colcon workspace and clone the projects
WORKDIR /colcon_ws/src
RUN git clone https://github.com/babakhani/rplidar_ros2.git && \
    git clone https://github.com/DJacquemont/articubot_one.git && \
    git clone --branch humble https://github.com/DJacquemont/diffdrive_arduino.git && \
    git clone https://github.com/joshnewans/serial.git

# Build the serial library
WORKDIR /colcon_ws/src/serial
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; make"
RUN make install

# Optimizing the .bashrc entries
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /colcon_ws/install/setup.bash" >> /root/.bashrc && \
    echo 'cd() { builtin cd "$@" && ls; }' >> /root/.bashrc

# Build the workspace
WORKDIR /colcon_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install"

# Set up the entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
