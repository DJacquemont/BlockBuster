#!/bin/bash

cd /
source /root/colcon_ws/install/setup.bash

chmod +x start_mission.sh

exec "$@"