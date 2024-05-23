#!/bin/bash
set -e

# Source the ROS 2 setup script
source /opt/ros/jazzy/setup.bash
source /opt/ros/env/bin/activate
source /root/ros2_ws/install/setup.bash

# Run Flask in the background
flask --app /root/flask-ws/src/index run --host=0.0.0.0 --port=5000 &

flask --app /root/flask-ws/src/updater run --host=0.0.0.0 --port=6000 &

# Run the ROS 2 launch file
ros2 launch cpp_parameters cpp_parameters_launch.py

# Wait for background processes
wait -n

# Exit with status of the first process to exit
exit $?