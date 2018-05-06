#!/bin/bash

echo "[INFO] Launching mavros_node"
roslaunch robot_localization sensor_fusion_0.launch &
sleep 25

echo "[INFO] Launching robot_localization nodes"
roslaunch robot_localization sensor_fusion_1.launch &

sleep 2
echo "[INFO] Launching ros0xrobot, rplidar, controller and rviz"
roslaunch robot_localization sensor_fusion_2.launch

wait