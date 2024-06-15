#!/bin/bash

cp colcon_ws/src/blockbuster_core/config/bt.xml /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/

source /opt/ros/humble/setup.bash
source /root/colcon_ws/install/setup.bash

exec "$@"