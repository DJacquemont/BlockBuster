#!/bin/bash

echo -e "\n#######################################################\n"
echo "                        SIMU MENU"
echo -e "\n#######################################################\n"
echo "Please choose an option:"
echo "1) Start the robot naked"
echo "2) Start the robot with SLAM"
echo "3) Start the robot with navigation & localisation"
echo "4) Build project"
echo -e "Enter your choice (1/2/3/4): \c"
read choice

echo "\nSearching for colcon_ws directory..."
colcon_dir=$(find /home -type d -name "colcon_ws" 2>/dev/null | head -n 1)
if [[ -z "$colcon_dir" ]]; then
    echo "colcon_ws directory not found."
    exit 1
else
    echo "colcon_ws directory found."
fi

if [ ! -f "$colcon_dir/install/setup.bash" ]; then
    echo "File install/setup.bash not found."
    exit 1
else
    source "$colcon_dir/install/setup.bash"
    echo "File install/setup.bash sourced."
fi

case $choice in
    1)
        echo -e "\nStarting the robot naked..."
        source /home/djacquem/colcon_ws/install/setup.bash
        echo -e "Configuration: SLAM disabled, Navigation disabled, Localisation disabled\n"
        ros2 launch articubot_one launch_sim.launch.py activate_slam:=false activate_nav:=false activate_loc:=false
        ;;
    2)
        echo -e "\nStarting the robot with SLAM..."
        source /home/djacquem/colcon_ws/install/setup.bash
        echo -e "Configuration: SLAM enabled, Navigation disabled, Localisation disabled\n"
        ros2 launch articubot_one launch_sim.launch.py activate_slam:=true activate_nav:=false activate_loc:=false
        ;;
    3)
        echo -e "\nStarting the robot with navigation & localisation..."
        source /home/djacquem/colcon_ws/install/setup.bash
        echo -e "Configuration: SLAM disabled, Navigation enabled, Localisation enabled\n"
        ros2 launch articubot_one launch_sim.launch.py activate_slam:=false activate_nav:=true activate_loc:=true
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
