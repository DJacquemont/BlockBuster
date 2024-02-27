# BlockBuster
Code implementation for our robot competing in the EPFL Robotic Competition (MAKE Project 10 ECTS).

## Getting Started

### Docker Container

A Docker container allows to ship, test, and deploy our applications in any environment without worrying about incompatible issues regardless of the machine's configuration settings.

This repository's Dockerfile sets up a development environment for ROS2 (Robot Operating System 2). It starts with a base ROS image and installs necessary tools and ROS2 packages. It creates a workspace, clones a GitHub project, and updates configurations.

The Docker container is automatically built for both **amd64** and **arm64** architecture when a push or a pull request happens on the main. If needed to build it manually (only works for your computer's architecture):
```
docker build -t <image_name> .
```

To pull from DockerHub the image `jacquemont/robot_os`:
```
docker pull jacquemont/robot_os
```

To run an image in a container, with vizualisation enabled:
```
xhost +

docker run --rm --privileged -it --net=host -e DISPLAY -v /dev/bus/usb:/dev/bus/usb <image_name> bash
```

To open a new terminal in the same container:
```
docker exec -it <container_name> bash
```

The container name (or container ID) can be found with `docker ps`, this command displays all the containers (active and inactive). In all terminal of the docker container, ros and the built package(s) are already built and sourced thanks to the **.bashrc** file.

#### 1. Demo LIDAR

To run the LIDAR's demo, please refer to the [rplidar_ros2](https://github.com/babakhani/rplidar_ros2) repository.

#### 2. Robot Simulation (ONLY for AMD64 architecture)

Start a teleoperation node in a terminal from the `colcon_ws` repository:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __node:=teleop_node --params-file install/articubot_one/share/articubot_one/config/keyboard.yaml -r /cmd_vel:=/cmd_vel_key
```

Start the simulation launch file in a new terminal:
```
ros2 launch articubot_one launch_sim.launch.py
```
Gazebo should automatically open. Note that Gazebo takes a long time to launch the first time it is opened. 

Start Rviz in a new terminal from the `colcon_ws` repository with the right configuration:
```
rviz2 -d src/articubot_one/config/main.rviz
```
Open from the GUI the configuration file `main.rviz` to monitor the robot.

To start SLAM with the `slam_toolbox` ros2 package from the `colcon_ws` repository:
```
ros2 launch slam_toolbox online_async_launch.py params_file:=src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=true
```
With the `slam_toolbox` can be created a map that will be later used for localization during navigation

#### 3. ROS2 Gazebo World 2D Map Generator

To create a 2D map from a Gazebo 3D world, package `gazebo_map_creator` can be used.

Terminal 1, from the `colcon_ws` repository:
```
gazebo -s libgazebo_map_creator.so src/articubot_one/worlds/obstacles.world
```

Terminal 2, from the `colcon_ws` repository:
```
ros2 run gazebo_map_creator request_map.py -c '(-4.8,-4.5,0.03)(4.8,4.5,1.0)' -r 0.01 -f $PWD/map
```

This second command create a map of the Gazebo world that can be found in the `colcon_ws`.

#### 4. Navigation

To launch AMCL (localization):
```
ros2 launch nav2_bringup localization_launch.py map:=./my_map_save.yaml use_sim_time:=true
```

To launch start the navigation:
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```


### Relevant Commands for ROS2

To build and source manually the ros package(s), in the `colcon_ws/` folder:
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install

source /colcon_ws/install/setup.bash
```
To start RQT, run `rqt` in a sourced terminal.
