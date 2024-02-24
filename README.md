# BlockBuster
Code implementation for our robot competing in the EPFL Robotic Competition (MAKE Project 10 ECTS).

## Getting Started

### Docker Container

A Docker container allows to ship, test, and deploy our applications in any environment without worrying about incompatible issues regardless of the machine's configuration settings.

This repository's Dockerfile sets up a development environment for ROS2 (Robot Operating System 2). It starts with a base ROS image and installs necessary tools and ROS2 packages. It creates a workspace, clones a GitHub project, and updates configurations.

The Docker container is automatically built for both **amd64** and **arm64** architecture when a push or a pull request happens on the main, and should not need to be built manually.

To pull from DockerHub and run the docker image:
```
docker pull jacquemont/robot_os

xhost +
docker run --rm --privileged -it --net=host -e DISPLAY -v /dev/bus/usb:/dev/bus/usb jacquemont/robot_os bash
```

To open a new terminal in the same container:
```
docker exec -it <container ID> bash
```

The container ID can be found with `docker ps`, this command displays all the containers (active and inactive). In all terminal of the docker container, ros and the built package(s) are already built and sourced thanks to the **.bashrc** file.

To run the LIDAR's demo, please refer to the [rplidar_ros2](https://github.com/babakhani/rplidar_ros2) repository.

### Relevant Commands for ROS2

To build and source manually the ros package(s), in the `colcon_ws/` folder:
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install

source /colcon_ws/install/setup.bash
```
To start RQT, run `rqt` in a sourced terminal.
