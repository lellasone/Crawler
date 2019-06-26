#!/bin/sh

sleep 1
source /opt/ros/kinetic/setup.sh
source /home/jake/Crawler/software/catkin_ws/devel/setup.sh


sleep 1

roscore	&

sleep 1

roslaunch spacenav_node classic.launch &

rosrun crawler_control chatter.py&
rosrun crawler_control speed.py&
rosrun crawler_control steer.py&

while true; do
	sleep 1m;
done
