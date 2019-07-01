#!/bin/sh

sleep 1
source /opt/ros/kinetic/setup.sh
source /home/jake/Crawler/software/catkin_ws/devel/setup.sh


sleep 1

roscore	&

sleep 1

roslaunch spacenav_node classic.launch &

roslaunch crawler_control standard.launch &

while true; do
	sleep 1m;
done
