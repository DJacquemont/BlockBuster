#!/bin/bash

cd /root
cp colcon_ws/src/blockbuster_core/config/bt.xml /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/
exec "$@"