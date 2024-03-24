#!/bin/bash

echo -e "\n#######################################################\n"
echo    "                  ROBOT IS STARTING..."
echo -e "\n#######################################################\n"

cd /root/colcon_ws
source /root/colcon_ws/install/setup.bash
ros2 launch articubot_one launch_robot.launch.py

exec "$@"