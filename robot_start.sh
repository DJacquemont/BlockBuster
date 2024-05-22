#!/bin/bash

echo -e "\n#######################################################\n"
echo "                       ROBOT MENU"
echo -e "\n#######################################################\n"
echo "Please choose an option:"
echo "1) Start the robot naked"
echo "2) Start the robot with SLAM"
echo "3) Start the robot with navigation & localisation"
echo "4) Build project"
echo -e "Enter your choice (1/2/3/4): \c"
read choice

colcon_dir="colcon_ws"
source "$colcon_dir/install/setup.bash"

case $choice in
    1)
        echo -e "\nStarting the robot naked..."
        echo -e "Configuration: SLAM disabled, Navigation disabled, Localisation disabled\n"
        ros2 launch blockbuster_core launch_robot.launch.py activate_slam:=false activate_nav:=false activate_loc:=false  activate_cam:=false  activate_sm:=false
        ;;
    2)
        echo -e "\nStarting the robot with SLAM..."
        echo -e "Configuration: SLAM enabled, Navigation disabled, Localisation disabled\n"
        ros2 launch blockbuster_core launch_robot.launch.py activate_slam:=true activate_nav:=false activate_loc:=false  activate_cam:=false  activate_sm:=false
        ;;
    3)
        echo -e "\nStarting the robot with navigation & localisation..."
        echo -e "Configuration: SLAM disabled, Navigation enabled, Localisation enabled\n"
        ros2 launch blockbuster_core launch_robot.launch.py activate_slam:=false activate_nav:=true activate_loc:=true  activate_cam:=true  activate_sm:=true
        ;;
    4)
        echo -e "\nBuilding project..."
        cd "$colcon_dir"
        colcon build
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac
