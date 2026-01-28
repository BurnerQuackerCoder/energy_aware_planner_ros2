#!/bin/bash
# 1. Setup environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Force UDP and disable shared memory for WSL2
export FASTRTPS_DEFAULT_PROFILES_FILE=~/noshm_fastdds.xml
export RMW_FASTRTPS_USE_CENTRAL_CONFIG=1
export ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE=1

echo "Launching Navigation Backend..."
# Use a subshell to ensure environment variables are inherited
(export FASTRTPS_DEFAULT_PROFILES_FILE=~/noshm_fastdds.xml; ros2 launch energy_aware_planner demo.launch.py) &

echo "Waiting 10 seconds for Lifecycle Nodes..."
sleep 10

# Force activate if nodes are stuck
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate

echo "Launching RViz..."
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/energy_aware_planner/config/energy_demo.rviz