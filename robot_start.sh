#!/bin/bash

echo -e "\n#######################################################\n"
echo "                       ROBOT MENU"
echo -e "\n#######################################################\n"
echo "Please choose an option:"
echo "1) Build project"
echo "2) Start the robot with SLAM"
echo "3) Start the robot without SLAM"
echo -e "Enter your choice (1/2/3): \c"
read choice

case $choice in
    1)
        colcon_dir=$(find / -type d -name "colcon_ws" 2>/dev/null | head -n 1)

        if [[ -z "$colcon_dir" ]]; then
            echo "colcon_ws directory not found."
            exit 1
        else
            echo -e "\nBuilding project..."
            cd "$colcon_dir"
            colcon build
        fi
        ;;
    2)
        echo -e "\nStarting the robot with SLAM..."
        source /root/colcon_ws/install/setup.bash
        echo -e "Configuration: SLAM enabled\n"
        ros2 launch articubot_one launch_robot.launch.py activate_slam:=true
        ;;
    3)
        echo -e "\nStarting the robot without SLAM..."
        source /root/colcon_ws/install/setup.bash
        echo -e "Configuration: SLAM disabled\n"
        ros2 launch articubot_one launch_robot.launch.py activate_slam:=false
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac