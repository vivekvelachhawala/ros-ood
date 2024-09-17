#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun ros_package motor_control_node.py

# wait for app to end
dt-launchfile-join
