# BlockBuster
Code implementation for our robot competing in the EPFL Robotic Competition (MAKE Project 10 ECTS).

## Getting Started

To pull and run the docker image:
```
docker pull jacquemont/robot_os

xhost +
docker run --rm --privileged -it --net=host -e DISPLAY -v /dev/bus/usb:/dev/bus/usb jacquemont/robot_os bash
```
Once inside the docker container, ros and the built package(s) are already sourced thanks to the entrypoint.sh file.

To open a new terminal in the same container:
```
docker exec -it <container ID> bash
```
The container ID can be found with `docker ps`, this command displays all the containers (active and inactive).

To run the LIDAR's demo, please refer to the [rplidar_ros2]([/guides/content/editing-an-existing-page](https://github.com/babakhani/rplidar_ros2)https://github.com/babakhani/rplidar_ros2) repository.


