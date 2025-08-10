#!/bin/bash

# Run controller node
echo "Running controller..."
cd /root/workspace
colcon build
source install/setup.bash
ros2 run limo_control controller &
CONTROLLER_PID=$!

# launch simulator in background
cd ..
echo "Launching sim in background..."
nohup ros2 launch limo_simulation limo.launch.py &
SIM_PID=$!

echo "Pausing before updating target pose..."
sleep 5

# Update target pose:
echo "Sending target update..."
ros2 topic pub /target geometry_msgs/msg/Pose2D "{x: 10.0, y: 10.0, theta: 1.571}" --once
wait