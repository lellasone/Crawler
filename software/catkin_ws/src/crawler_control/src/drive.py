#! /usr/bin/env python3

# This module is responsible for interpreting commands from various
# control sources (joystick, vector3, ect..) and turning them into 
# velocity and heading commands for the robot. This module should 
# be started before the steer and speed modules. 


import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import time
import math
from ackermann_msgs.msg import AckermannDriveStamped


INDEX_SPEED_SPACENAV = 0
INDEX_STEER_SPACENAV = 5
INDEX_SPEED_PS4 = 1
INDEX_STEER_PS4 = 0


TEST_SPEED = 2 #m/s
TURN_MAX = 0.5 # maximum turning angle in radians
RADIUS_MIN = 0.667 # min turning radius in meters


SPEED_GEAR_RATIO = 8.11*(54/18) # motor rotations per wheel rotation (8.11 * spur/ pinion). 
SPEED_WHEEL_DIAMETER = 0.2 # wheel diameter in meters. 
SPEED_MAX_RPM = 3500 # Maximum allowable speed. 

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
	global velocity
	global cruse_control
	if(msg.buttons[0] == 1):
		cruse_control = True
	elif(msg.buttons[1] == 1):
		cruse_control = False

	if not cruse_control:
		velocity = msg.axes[INDEX_SPEED_SPACENAV] * SPEED_MAX_RPM
	else:
		velocity = convert_velocity(TEST_SPEED)

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


def listener_ackermann_auto():
	''' 
		This function is responsible for listening to incomming movment
		commands from the ps4 controller and responding to them. 
	'''
	rospy.Subscriber("/ackermann_cmd_openloop", AckermannDriveStamped, callback_ackermann)
	rospy.spin()

def callback_twist(msg):
	'''
		TODO: finish function.

		This function is responsible interpreting twist messages in SI units and 
		setting the speed and steering setpoints appropriatly. 
	'''
	heading_rad = msg.radius.z # desired angle in radians
	velocity_si = msg.linear.x # desired velocity in m/s

	# pin steering to the rails if needed. 
	if heading_rad > TURN_MAX:
		heading_rad = TURN_MAX
	elif heading_rad < -1 * TURN_MAX:
		heading_rad = -1 * TURN_MAX

	velocity_rpm = convert_velocity(velocity_si)
	steer_angle = convert_angle(heading_rad)

def callback_ackermann(ack_msg):
	'''
		TODO: finish function.

		This function is responsible interpreting ackermann messages in SI units
		and setting the speed and steering setpoints appropriately. 
	'''
	heading_rad = ack_msg.drive.steering_angle # desired angle in radians
	velocity_si = ack_msg.drive.speed # desired velocity in m/s

	# pin steering to the rails if needed. 
	if heading_rad > TURN_MAX:
		heading_rad = TURN_MAX
	elif heading_rad < -1 * TURN_MAX:
		heading_rad = -1 * TURN_MAX

	velocity_rpm = convert_velocity(velocity_si)
	steer_angle = convert_angle(heading_rad)

	set_movement_ackermann(velocity_rpm, steer_angle)
	

def convert_velocity(velocity):
	'''
		Converts the input velocity in meters per second, into an angular 
		velocity in RPM at the motor. This will depend on the robot and
		requires that the corresponding paramiters be set at the top of 
		this file. 
		args:
			velocity - The desired robot velocity in m/s. 
	'''
	meters_per_minute = velocity * 60.0 
	rpm_wheel = meters_per_minute / (math.pi * SPEED_WHEEL_DIAMETER)
	rpm_motor = rpm_wheel * SPEED_GEAR_RATIO
	return(rpm_motor)

def convert_angle(desired_angle):
	'''
		Converts the input desired angle in radius, into the angle for 
		the steering in radians. This will depend on the robot and
		requires that the corresponding paramiters be set at the top of 
		this file. 
		args:
			desired_angle - The desired robot angle direction in radians. 
		Returns: 
			Desired angle scaled to -0.5 to 0.5 corresponding servo movment range
			(notably this likely exceeds the steering control range. )
	'''
	steer_angle = desired_angle / TURN_MAX # scale to -1 to 1 range

	return(steer_angle)

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

	commands_speed.publish(msg.axes[index_speed]* SPEED_MAX_RPM)
	commands_steer.publish(msg.axes[index_steer])

def set_movement_ackermann(velocity_rpm, steer_angle):
	'''
		This function handles new steering and speed requests from ackermann automomy. 
		It should be called each time a new ackwrmann message is recieved.  

		args:
			velocity_rpm - requested speed of the motor in rpm
			steer_angle - requested steering angle of the wheels in radians
	'''

	commands_speed.publish(velocity_rpm)
	commands_steer.publish(steer_angle)

def steer_angle_test(velocity_si, steer_angle):
	''' 
		This function drives the robot at continuous linear velocity at same steering 
		angle for 4 seconds to  get the steering angle correlation to real world angle.

		args:
			velocity_si - requested velocity in m/s
			steer_angle - requested steering angle of the wheels in radians
	'''
	t0 = time.time()
	while ((time.time() - t0) < 4.0):
		commands_speed.publish(convert_velocity(velocity_si))
		commands_steer.publish(-steer_angle):
	commands_speed.publish(convert_velocity(0.0))
	commands_steer.publish(0)


if __name__ == '__main__':
	init_node()
	#listener_spacenav_joy()
	#listener_ps4_joy()
	listener_ackermann_auto()
	#spin_listeners()

