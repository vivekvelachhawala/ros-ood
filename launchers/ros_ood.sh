#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber

dt-exec rosrun ros_package ood_node.py
sleep 20
dt-exec rosrun ros_package lane_following_node.py
sleep 10
dt-exec rosrun ros_package motor_control_node.py


# wait for app to end
dt-launchfile-join
