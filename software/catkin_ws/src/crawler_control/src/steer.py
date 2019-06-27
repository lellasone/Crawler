#!/usr/bin/env python

# This module is responcible for the Crawler's stearing system. It is run when
# the robot is first booted. 
#
# Maintainer: Jake Ketchum, jketchum@caltech.edu

import rospy
from sensor_msgs.msg import Joy
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
def send_frame(command, data, reply_length = 0):
	try: 
	    teensy = serial.Serial(port)
	    frame = START_BYTE
	    frame = frame + command + data
	    frame = frame + bytearray.fromhex("00") # add checksum
	    frame = frame + END_BYTE
	    
	    print(frame)
	    teensy.write(frame)
	    if reply_length != 0:
	        return [True, (binascii.hexlify(teensy.read(reply_length)))]
	except: 
		print("serial exception")
def request_echo(payload):
    responce = send_frame(COMMAND_ECHO, payload, 2)
    return responce 

# Sends a steering setpoint update command to the teensy. 
# args: 
#	payload - a single byte with the neutral position fo the wheels set at 128. 
def set_steering_setpoint(payload):
	if (type(payload) != bytes):
		print("ERROR: steering setpoint must be a bytes")
		print("Is Type: " + str(type(payload)))
		return [False, 0] #indicate an error occured. 
	if (len(payload) != 2):
		print("ERROR: steering setpoint must be a byte array of length 2")
		return [False, 0] #indicate an error occured. 
	responce = send_frame(COMMAND_STEER, payload, 0)
	return responce


# This function is responcible for repeatedly transmitting the current 
# steering setpoint to the ppm controller. 
def send_steering():
	rate = rospy.Rate(50)
	while True: 

		set_steering_setpoint(bytes([setpoint, 0]))
		rate.sleep()
		print(setpoint)



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
	rospy.init_node('steer', anonymous = True)
	commander = threading.Thread(target=send_steering)
	commander.start()
	listener()


