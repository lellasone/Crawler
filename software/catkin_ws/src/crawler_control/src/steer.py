#!/usr/bin/env python3

# This module is responcible for the Crawler's stearing system. It is run when
# the robot is first booted. 
#
# Maintainer: Jake Ketchum, jketchum@caltech.edu

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Float64
import threading 

import serial
import time
import binascii
import os

COMMAND_ECHO = bytearray.fromhex("62")
COMMAND_PING = bytearray.fromhex("61")
COMMAND_STEER = bytearray.fromhex("63")
START_BYTE = bytearray.fromhex("41")
END_BYTE = bytearray.fromhex("5A")

port = "/dev/ttyACM0"


INDEX_PAN = 3
INDEX_SPEED = 4

STEER_MAX = 0.4835 # maximum steering possible angle in radians 

SETPOINT_MIN = 60 # smallest meaningful output value. 
SETPOINT_MAX = 190 # Largest meaningful output value. 

setpoint = 128 # the steering setpoint. 



# This function composes and transmits each command frame. 
# It must be provided with the command byte, data payload,
# and length of expected reply (if appropriate)
# This function will return a list in the form [bool, responce] where 
# bool is True if the communication was succesful, and False if it failed. 
# if a length of 0 is specified for the responce that index will be set at ""
def send_frame(command, data, reply_length = 0, port_id = None):
	
	# if no port has been provided use the global default. 
	if port_id == None: 
		port_id = port

	try: 
	    teensy = serial.Serial(port_id)
	    frame = START_BYTE
	    frame = frame + command + data
	    frame = frame + bytearray.fromhex("00") # add checksum
	    frame = frame + END_BYTE
	    
	    #rospy.loginfo(frame)
	    teensy.write(frame)
	    if reply_length != 0:
	        return [True, teensy.read(reply_length)]
	    else:
	    	return [True, ""]
	except serial.SerialException as e: 
		rospy.logerr("serial exception: " + str(e))
		return[False,""]
		

def request_echo(payload):
	'''
		This function requests that the target device transmit back the 
		payload bytes exactly as they are recieved. This can be used to 
		verify that the device is functioning or for stress testing of 
		the serial system. 
	'''
	responce = send_frame(COMMAND_ECHO, payload, 2)
	return responce 

# Requests the device ID of the target device. 
def request_ping(port_id = port):
	responce = send_frame(COMMAND_PING, bytearray.fromhex("0000"), 3, port_id = port_id)
	return responce

# Sends a steering setpoint update command to the teensy. 
# args: 
#	payload - a pair of  bytes with the neutral position of the 
#			  wseels set at 128. The second byte should always 
#			  be zero. 
def set_steering_setpoint(payload):
	if (type(payload) != bytes):
		rospy.logerr("ERROR: steering setpoint must be a bytes")
		rospy.logerr("Is Type: " + str(type(payload)))
		return [False, 0] #indicate an error occured. 
	if (len(payload) != 2):
		rospy.logerr("ERROR: steering setpoint must be a byte array of length 2")
		return [False, 0] #indicate an error occured. 
	responce = send_frame(COMMAND_STEER, payload, 0)
	global count
	count = 0
	return responce


# This function is responcible for repeatedly transmitting the current 
# steering setpoint to the ppm controller. 
def send_steering():
	'''
		This function is meant to be called as a seperate thread. It
		repeatedly polls the steering setpoint and transmits that setpoint
		to the teensy. 

		This function is also responsible for monitoring the time since
		a new input was detected, and for setting the steering to 
		a neutral position if that time delay beomes to long. 
	'''
	rate = rospy.Rate(50) 
	global failed_count
	failed_count = 0 # variable to check if count was reset to 0 after a fail
	global count
	count = 1
	while True: 
		rate.sleep()
		if count != 0 and failed_count == 0:
			responce = set_steering_setpoint(bytes([setpoint, 0]))
			failed_count += 1
		elif count == 0:
			count += 1
			failed_count = 0
		else: # if  count != 0 and failed count != 0, means lost control
			# stop the robot
			pass
		if not responce[0]:
			rospy.logwarn("steering update failed")


# This function scans through all of the avaliable ACM ports (0 - 9) and 
# attempts to ping each one. The lowest value'd device that matches the specified
# ID string is the one returned. If no device is found an empty string will be 
# returned. If a device ID is found, it will also be set as the device ID for this
# serial parser. 
# Args: 
# 	ID - The ID of the device to be found, given as a string. 
def scan_ports(ID):
	for i in range (0, 10):
		port_i = "/dev/ttyACM" + str(i)
		responce = request_ping(port_id = port_i)[1]
		if ID == responce:
			rospy.loginfo("setting steering port to: " + str(port_i))
			global port 
			port = port_i
			return(port)
	return("")



def listener():
	rospy.Subscriber("crawler/command_steer", Float64, callback)
	
	print("spinning up listener")
	rospy.spin()
	print("listener down")


def callback(msg):
	'''
		This function is called every time a new steering request is recieved
		from the drive system. It updates the speed setpoint as appropriate
		but does not transmit that setpoint to the teensy. 

		Note: 	This function needs to recieve a value between -1 and 1,
				where 0 corresponds to straight and +1 corresponds to a 
				left turn. 

		TODO: Redo for twist (rads to servo angle)
	'''
	angle = msg.data# scale to range -1 to 1
	if(angle <= 1 and angle >= -1):
		angle += 1 # scake to range 0 to 2 
		angle = angle * 127 # scale to 8 bits. (0 to 255)
		global setpoint 
		setpoint = int(angle)
	else:
		rospy.logerr("setpoint must be between -1 and 1, is currently: " + str(angle))

def spin_send_steering():
	rospy.init_node('steer', anonymous = True)
	commander = threading.Thread(target = send_steering)
	commander.start()


if __name__ == '__main__':
	print(scan_ports(bytes(bytearray.fromhex("4A504C"))))
	print(port)
	time.sleep(5)
	
	spin_send_steering()
	listener()


