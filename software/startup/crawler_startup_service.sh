#!/bin/sh

sleep 1
source /opt/ros/kinetic/setup.sh

sleep 1

roscore	&

sleep 1

roslaunch spacenav_node classic.launch &

gnome-terminal --tab="speed" --command="bash -c 'python3.6 ~/Crawler/software/catkin_ws/scripts/drive.py; $SHELL'" --tab="steer" --command="bash -c 'python3.6 ~/Crawler/software/catkin_ws/scripts/steer.py;$SHELL'"&
 &

while true; do
	sleep 1m;
done
