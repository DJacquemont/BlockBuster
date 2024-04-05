# BlockBuster

Code implementation for our robot competing in the EPFL Robotic Competition (MAKE Project 10 ECTS).

<p align="center">
  <img src="images/coolgif.gif" width="30%">
</p>

## Installation

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11
- Python 3.10

### 1. Nvidia Jetson Nano Connection

1. Connect to the zerotier network (ID: `856127940c82f9a4`):
```
sudo zerotier-cli join 856127940c82f9a4
```

2. SSH into the Jetson Nano:
```
ssh jetson@nano.local
psswd: jetson
```

3. Shutdown Jetson Nano:
```
sudo poweroff
```

### 2. Docker Container Setup (Run ROS2 on Jetson Nano)

1. OPTIONAL - Pull the Docker image from Docker Hub (if not already done):
```
docker pull jacquemont/robot_os
```

2. Run the Docker container:
```
sudo docker run --rm --privileged -it --net=host -v /dev/bus/usb:/dev/bus/usb -v /home/jetson/colcon_ws/src:/root/colcon_ws/src jacquemont/robot_os bash
```

3. Open the robot software startup menu in the docker container:
```
bash robot_start.sh
```

4. OPTIONAL - Open a new terminal in the Docker container:
```
sudo docker exec -it $(sudo docker ps -aqf "ancestor=jacquemont/robot_os") bash
```

4. Exit Docker container:
```
ctrl -D
```

### 3. Local Machine Setup (Run ROS2 locally)

1. Create a new workspace:
```
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
```

2. Clone the repository:
```
git clone git@github.com:DJacquemont/blockbuster.git .
```

3. Fetch the submodules:
```
git submodule update --init --recursive
```

4. Build the workspace:
```
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

5. Source the workspace:
```
source ~/colcon_ws/install/setup.bash
```
Once the workspace is built, the ROS2 nodes can be run.

## Getting Started

### 1. Robot HARDWARE

To start the robot hardware (once the robot's hardware is connected to the laptop), run the following command from the `colcon_ws` repository:
```
ros2 launch articubot_one launch_robot.launch.py
```

### 2. Robot SIMULATION (ONLY for AMD64 architecture)

Start the simulation launch file in a new terminal:
```
ros2 launch articubot_one launch_sim.launch.py
```
Gazebo should automatically open. Note that Gazebo takes a long time to launch the first time it is opened. 

### 3. Other Nodes

#### 3.1. Teleoperation

Start a teleoperation node in a terminal from the `colcon_ws` repository:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __node:=teleop_node -r /cmd_vel:=/cmd_vel_key
```

#### 3.2. Rviz

Start Rviz in a new terminal from the `colcon_ws` repository with the right configuration:
```
rviz2
```
Open from the GUI the configuration file `main.rviz` to monitor the robot.

#### 3.3. SLAM

To start SLAM with the `slam_toolbox` ros2 package from the `colcon_ws` repository:
```
ros2 launch slam_toolbox online_async_launch.py params_file:=src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=true
```
With the `slam_toolbox` can be created a map that will be later used for localization during navigation

#### 3.4. Navigation

To launch AMCL (localization):
```
ros2 launch nav2_bringup localization_launch.py map:=./my_map_save.yaml use_sim_time:=true
```

To start the navigation:
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```
