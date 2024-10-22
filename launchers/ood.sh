#!/bin/bash

source /environment.sh

export LD_PRELOAD=/usr/local/lib/python3.8/dist-packages/torch.libs/libgomp-804f19d4.so.1.0.0

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun ros_package ood_node.py

# wait for app to end
dt-launchfile-join
