#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun ros_package lane_following_node.py

# wait for app to end
dt-launchfile-join
