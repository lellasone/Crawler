#! /usr/bin/env python3

# This module is responsible for interpreting commands from various
# control sources (joystick, vector3, ect..) and turning them into 
# velocity and heading commands for the robot. This module should 
# be started before the steer and speed modules. 


import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import time


INDEX_SPEED_SPACENAV = 0
INDEX_STEER_SPACENAV = 5
INDEX_SPEED_PS4 = 1
INDEX_STEER_PS4 = 0

def init_node():
	''' 
	'''
	global commands_speed
	global commands_steer
	commands_speed = rospy.Publisher("crawler/command_speed", Float64, queue_size = 1)
	commands_steer = rospy.Publisher("crawler/command_steer", Float64, queue_size = 1)

	rospy.init_node("crawler")


def listener_spacenav_joy():
	''' 
		This function is responsible for listening to incomming movment
		commands from the spacenav controller and responding too them. 
	'''
	rospy.Subscriber("/spacenav/joy", Joy, callback_spacenav)
	rospy.spin()

def callback_spacenav(msg):
	set_movement_joy(msg, INDEX_SPEED_SPACENAV, INDEX_STEER_SPACENAV)
	pass

def listener_ps4_joy():
	''' 
		This function is responsible for listening to incomming movment
		commands from the ps4 controller and responding to them. 
	'''
	rospy.Subscriber("/ps4/joy", Joy, callback_ps4)
	rospy.spin()

def callback_ps4(msg):
	print("ping")
	set_movement_joy(msg, INDEX_SPEED_PS4, INDEX_STEER_PS4)


def set_movement_joy(msg, index_speed, index_steer):
	'''
		This function handles new steering and speed requests from joystics. 
		It should be called each time a new joystick message is recieved. The
		calling function must specifiy which axes corespond to speed and heading. 

		Note that this function does not handle button pushes.
		args:
			msg - a sensor_msgs.msg.Joy message to be parsed. 
			index_speed - the index of the speed axis. 
			index_steer - the index of the steer axis. 
	'''
	commands_speed.publish(msg.axes[index_speed])
	commands_steer.publish(msg.axes[index_steer])
	


if __name__ == '__main__':
	init_node()
	#listener_spacenav_joy()
	listener_ps4_joy()
	#spin_listeners()

