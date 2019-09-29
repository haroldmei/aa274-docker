#!/bin/bash

if [[ -z $build ]]; then
	build=devel
fi

# Setup ros environment
source /opt/ros/kinetic/setup.bash
if [[ -f ~/catkin_ws/$build/setup.bash ]]; then
	source ~/catkin_ws/$build/setup.bash
fi

# Export environment variables
if [[ -z $ROS_MASTER_URI ]]; then
	export ROS_MASTER_URI=http://master:11311
fi
export TURTLEBOT3_MODEL=waffle
