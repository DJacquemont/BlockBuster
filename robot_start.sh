#!/bin/bash

# # Check if /root/colcon_ws/install exists
# if [ ! -d "~/colcon_ws/install" ]; then
#     echo "The /root/colcon_ws/install directory does not exist. Building the project..."
#     # Navigate to the colcon_ws directory
#     cd ~/colcon_ws
#     # Build the project
#     colcon build
#     # After building, check if the build was successful
#     if [ ! -d "~/colcon_ws/install" ]; then
#         echo "Build failed. The ~/colcon_ws/install directory still does not exist."
#         exit 1
#     fi
# fi

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

case $choice in
    4)
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
        echo -e "Configuration: SLAM enabled, Navigation disabled, Localisation disabled\n"
        ros2 launch articubot_one launch_robot.launch.py activate_slam:=true activate_nav:=false activate_loc:=false
        ;;
    3)
        echo -e "\nStarting the robot with navigation & localisation..."
        source /root/colcon_ws/install/setup.bash
        echo -e "Configuration: SLAM disabled, Navigation enabled, Localisation enabled\n"
        ros2 launch articubot_one launch_robot.launch.py activate_slam:=false activate_nav:=true activate_loc:=true
        ;;
    1)
        echo -e "\nStarting the robot naked..."
        source /root/colcon_ws/install/setup.bash
        echo -e "Configuration: SLAM disabled, Navigation disabled, Localisation disabled\n"
        ros2 launch articubot_one launch_robot.launch.py activate_slam:=false activate_nav:=false activate_loc:=false
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac
