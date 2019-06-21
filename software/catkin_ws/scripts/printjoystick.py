#!/usr/bin/env python

# This is a test script for reading joystick data. It reads data from a 
# ros joystick topic and prints a version of that topic with the throttle
# stearing and stop commands to the terminal. 

import rospy
from sensor_msgs.msg import Joy

INDEX_PAN = 3
INDEX_SPEED = 4


def listener():
	print("Setting up listener")
	rospy.init_node('listener',anonymous=True)

	rospy.Subscriber("/spacenav/joy", Joy, callback)
	
	print("spinning up listener")
	rospy.spin()
	print("listener down")

def callback(msg):
	print("speed: " + str(msg.axes[INDEX_SPEED]))
	print("pan: " + str(msg.axes[INDEX_PAN]))

if __name__ == '__main__':
	listener()
