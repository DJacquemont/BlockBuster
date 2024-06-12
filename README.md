# BlockBuster

Code implementation for our robot competing in the EPFL Robotic Competition (MAKE Project 10 ECTS).

<p align="center">
  <img src="images/coolgif.gif" width="30%">
</p>

## Installation

### Prerequisites

Software requirements:
- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11
- Python 3.10
- Docker

Hardware requirements:
- Raspberry Pi 5 (or equivalent)


### 1. Onboard Computer Wireless Connection

1. OPTIONAL - Connect to the zerotier network (ID: `856127940c82f9a4`):
```
sudo zerotier-cli join 856127940c82f9a4
```

2. SSH into the onboard computer:
```
raspberry@172.27.27.207
psswd: pi
```

3. Shutdown onboard computer:
```
sudo poweroff
```
4. Disconnect from the zerotier network (ID: `856127940c82f9a4`):
```
sudo zerotier-cli leave 856127940c82f9a4
```

### 2. Docker Container Setup (Run ROS2 in Docker)

1. OPTIONAL - Pull the Docker image from Docker Hub (if not already done):
```
docker pull jacquemont/robot_os
```

2. Run the Docker container:
```
sudo docker run --rm --privileged -it --net=host -v /dev/bus/usb:/dev/bus/usb -v /home/raspberry/colcon_ws/src:/root/colcon_ws/src jacquemont/robot_os bash
```

3. Open the robot software startup menu in the docker container:
```
./robot_start.sh
```

4. OPTIONAL - Open a new terminal in the Docker container:
```
sudo docker exec -it $(sudo docker ps -aqf "ancestor=jacquemont/robot_os") bash
```

### 3. Local Machine Setup (Run ROS2 locally)

1. Create a new workspace:
```
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
```

2. Clone the repository with the submodules:
```
git clone --recursive git@github.com:DJacquemont/BlockBuster.git .
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

## Getting Started

### 1. Robot HARDWARE

To start the robot hardware, execute the file `robot_start.sh` once connected to the onboard computer, or run the following command:
```
ros2 launch blockbuster_core launch_robot.launch.py activate_slam:=false activate_nav:=true activate_loc:=true  activate_cam:=true  activate_sm:=true
```

### 2. Robot SIMULATION (ONLY for AMD64 architecture)

To start the robot simulation, run the following command:
```
ros2 launch blockbuster_core launch_sim.launch.py activate_slam:=false activate_nav:=true activate_loc:=true  activate_cam:=true  activate_sm:=true
```

### 3. Other Nodes

#### 3.1. Teleoperation

Start a teleoperation node in a terminal from the `colcon_ws` repository:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __node:=teleop_node -r /cmd_vel:=/cmd_vel_key
```

Or with the joystick:
```
ros2 launch blockbuster_core joystick.launch.py
```

#### 3.2. Rviz

Start Rviz in a new terminal from the `colcon_ws` repository with the right configuration:
```
rviz2
```
Open from the GUI the configuration file `main.rviz` to monitor the robot.
