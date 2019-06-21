#!/bin/sh

sleep 1
source /opt/ros/kinetic/setup.sh

sleep 1

roscore	&

sleep 1

roslaunch spacenav_node classic.launch &

python3 /home/jake/Crawler/software/catkin_ws/scripts/drive.py&
python3 /home/jake/Crawler/software/catkin_ws/scripts/steer.py&

while true; do
	sleep 1m;
done
