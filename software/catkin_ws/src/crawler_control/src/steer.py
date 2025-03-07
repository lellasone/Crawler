#!/usr/bin/env python3

# This module is responcible for the Crawler's stearing system. It is run when
# the robot is first booted. 
#
# Maintainer: Jake Ketchum, jketchum@caltech.edu

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import threading 

import serial
import time
import binascii
import os
import rospy

COMMAND_ECHO = bytearray.fromhex("62")
COMMAND_PING = bytearray.fromhex("61")
COMMAND_STEER = bytearray.fromhex("63")
START_BYTE = bytearray.fromhex("41")
END_BYTE = bytearray.fromhex("5A")

port = "/dev/ttyACM0"


INDEX_PAN = 3
INDEX_SPEED = 4

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
	    
	    rospy.loginfo(frame)
	    teensy.write(frame)
	    if reply_length != 0:
	        return [True, teensy.read(reply_length)]
	    else:
	    	return [True, ""]
	except serial.SerialException as e: 
		rospy.logerr("serial exception: " + str(e))
		return[False,""]
		

def request_echo(payload):
    responce = send_frame(COMMAND_ECHO, payload, 2)
    return responce 

# Requests the device ID of the target device. 
def request_ping(port_id = port):
	responce = send_frame(COMMAND_PING, bytearray.fromhex("0000"), 3, port_id = port_id)
	return responce

# Sends a steering setpoint update command to the teensy. 
# args: 
#	payload - a single byte with the neutral position of the wheels set at 128. 
def set_steering_setpoint(payload):
	if (type(payload) != bytes):
		rospy.logerr("ERROR: steering setpoint must be a bytes")
		rospy.logerr("Is Type: " + str(type(payload)))
		return [False, 0] #indicate an error occured. 
	if (len(payload) != 2):
		rospy.logerr("ERROR: steering setpoint must be a byte array of length 2")
		return [False, 0] #indicate an error occured. 
	responce = send_frame(COMMAND_STEER, payload, 0)
	return responce


# This function is responcible for repeatedly transmitting the current 
# steering setpoint to the ppm controller. 
def send_steering():
	rate = rospy.Rate(50)
	count = 0
	while True: 
		responce = set_steering_setpoint(bytes([setpoint, 0]))
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
	rospy.Subscriber("/spacenav/joy", Joy, callback)
	
	print("spinning up listener")
	rospy.spin()
	print("listener down")


def callback(msg):
	try: 
		angle = msg.axes[INDEX_PAN]
		angle = angle + 1 #re-center as positive
		angle = angle * 128 # scale to 8 bits. 
		global setpoint 
		setpoint = int(angle)
	except: 
		pass 


if __name__ == '__main__':
	print(scan_ports(bytes(bytearray.fromhex("4A504C"))))
	print(port)
	time.sleep(5)
	rospy.init_node('steer', anonymous = True)
	commander = threading.Thread(target=send_steering)
	commander.start()
	listener()


