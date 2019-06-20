#!/usr/bin/env python

# This is a test script for reading joystick data. It reads data from a 
# ros joystick topic and prints a version of that topic with the throttle
# stearing and stop commands to the terminal. 

import rospy
from std_msgs.msg import String


def listener():
	ropsy.init_node('listener',anonymous=True)

rospy.Subscriber("
