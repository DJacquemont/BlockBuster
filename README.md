# BlockBuster
Code implementation for our robot competing in the EPFL Robotic Competition (MAKE Project 10 ECTS)

To pull and run the docker image:
```
docker pull jacquemont/robot_os

xhost+
docker run --rm --privileged -it --net=host -e DISPLAY -v /dev/bus/usb:/dev/bus/usb jacquemont/robot_os bash
```
