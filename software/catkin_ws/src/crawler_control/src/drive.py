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

STEER_MAX = 0.5 #maximum turning angle in radians

SPEED_GEAR_RATIO = 5 # motor rotations per wheel rotation (on average). 
SPEED_WHEEL_DIAMETER = 0.2 # wheel diameter in meters. 

cruse_control = False # when true new velocity requests will be ingored. 
requested_velocity = 0 #velocity, used with cruse control. 

def init_node():
	'''
		This function creates the rosnodes used for commanding the robot. 
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
	if(msg.buttons[0] == 1):
		cruse_control = True
	elif(msg.buttons[1] == 1)
		cruse_control = False
	global velocity
	
	if not cruse_control:
		velocity = -1 * msg.axes[INDEX_SPEED_SPACENAV]

	commands_speed.publish(velocity)
	commands_steer.publish(msg.axes[INDEX_STEER_SPACENAV])

def listener_ps4_joy():
	''' 
		This function is responsible for listening to incomming movment
		commands from the ps4 controller and responding to them. 
	'''
	rospy.Subscriber("/ps4/joy", Joy, callback_ps4)
	rospy.spin()

def callback_ps4(msg):
	set_movement_joy(msg, INDEX_SPEED_PS4, INDEX_STEER_PS4)

def callback_twist(msg):
	'''
		TODO: finish function.

		This function is responsible interpreting twist messages in SI units and 
		setting the speed and steering setpoints appropriatly. 
	'''
	heading_rad = msg.angular.z # desired turn angle in radians. (relative to current)
	velocity_si = msg.linear.x # desired velocity in m/s

	# pin steering to the rails if needed. 
	if heading_rad > STEER_MAX:
		heading_rad = STEER_MAX
	elif heading_rad < -1* STEER_MAX:
		heading_rad = -1*STEER_MAX
	



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

