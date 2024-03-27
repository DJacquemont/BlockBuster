#!/bin/bash

source /root/colcon_ws/install/setup.bash

echo -e "\n#######################################################\n"
echo "                       ROBOT MENU"
echo -e "\n#######################################################\n"

# Prompt the user to decide on activating SLAM
read -p "Do you want to activate SLAM? (y/N) " activate_slam_response
echo -e "\nConfiguration:"

# Check the user's response
if [[ "$activate_slam_response" == "y" ]]; then
    echo -e "   SLAM enabled\n"
    ros2 launch articubot_one launch_robot.launch.py activate_slam:=true
else
    echo -e "   SLAM disabled\n"
    ros2 launch articubot_one launch_robot.launch.py activate_slam:=false
fi