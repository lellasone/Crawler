#!/bin/sh

sleep 1
source /opt/ros/kinetic/setup.sh

sleep 1

roscore	&

sleep 1

roslaunch spacenav_node classic.launch &

while true; do
	sleep 1m;
done
