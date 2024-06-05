#!/bin/bash

# Function to be executed upon receiving SIGINT (Ctrl+C)
cleanup() {
  echo "Received Ctrl+C, killing terminals..."
  kill $image_handler_pid
  kill $controller_pid
  kill $simulation_handler_pid
  exit 1
}

# Trap the SIGINT signal and call the cleanup function
trap cleanup SIGINT

echo "Building line_tracking_race"
catkin build line_tracking_race

source ~/catkin_ws/devel/setup.bash

echo "Initiating gazebo simulation"
gnome-terminal -- bash -c "source ~/catkin_ws/devel/setup.bash; roslaunch line_tracking_race race_track.launch"
simulation_handler_pid=$!

echo "Launching image_handler.py in a new terminal"
gnome-terminal -- bash -c "source ~/catkin_ws/devel/setup.bash; rosrun line_tracking_race image_handler.py;"
image_handler_pid=$!

echo "Launching controller.py in a new terminal"
gnome-terminal -- bash -c "source ~/catkin_ws/devel/setup.bash; sleep 10; rosrun line_tracking_race controller.py;"
controller_pid=$!

echo 
echo
echo "Waiting for CTRL+C to terminate execution"

# Wait indefinitely
while true; do
  sleep 1
done