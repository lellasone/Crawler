#!/usr/bin/env python3

# This module is reponsible for interfacing with the crawler's e-stop system. 
# It rely's on the higher level software e-stop system to send stop-commands
# when requested. This module is largley identical to the steering module.
#
#
# Note: responces will generally be in the form: [id][id][id][errors][data bytes...]
# 		But not all functions have been updated on the teensy so check. 
#
# Maintainer: Jake Ketchum, jketchum@caltech.edu

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import threading 
import termios

import serial
import time
import binascii
import os
import math

COMMAND_ECHO = bytearray.fromhex("62")
COMMAND_PING = bytearray.fromhex("61")
COMMAND_STEER = bytearray.fromhex("63")
COMMAND_WRITE_D = bytes("d", encoding = 'utf8')
COMMAND_READ_D = bytes("e", encoding = 'utf8')
START_BYTE = bytearray.fromhex("41")
END_BYTE = bytearray.fromhex("5A")
DEVICE_ID = bytes("AAA", encoding = 'utf8')

RESPONCE_READ_D_LN = 5 # length of read digital response in bytes. 
RESPONCE_INDEX_ID_S = 0 #start of responce bytes. 
RESPONCE_INDEX_ID_E = 2 #start of responce bytes. 
RESPONCE_INDEX_ERROR = 3# teensy errors if any. 
RESPONCE_INDEX_DATA = 4 #start of data bytes. 

PIN_TRANSPONDER = 8 # arduino number of transponder read pin. 


ESTOP_TOPIC = 'xmaxx/estop'

port = "/dev/ttyACM0"
SERIAL_TIMEOUT = 0.02 #read timeout in seconds. 



ALLOWED_FAILURES = 10 # allowed failed reads befor searching for new port.
UPDATE_RATE = 200 # rate of transmission in hz. 



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
	    teensy = serial.Serial(port_id, timeout=SERIAL_TIMEOUT)
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
		#rospy.logerr("serial exception: " + str(e))
		return[False,""]
	except termios.error as e: 
		rospy.logger("Termios exception: " + str(e))
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

# This function scans through all of the avaliable ACM ports (0 - 9) and 
# attempts to ping each one. The lowest value'd device that matches the specified
# ID string is the one returned. If no device is found an empty string will be 
# returned. If a device ID is found, it will also be set as the device ID for this
# serial parser. 
# Args: 
# 	ID - The ID of the device to be found, given as a string. 
def scan_ports(ID):
	for i in range (0, 10):
		try:
			port_i = "/dev/ttyACM" + str(i)
			responce = request_ping(port_id = port_i)[1]
			if ID == responce:
				rospy.loginfo("setting steering port to: " + str(port_i))
				global port 
				port = port_i
				return(port)
		except serial.SerialException as e:
			rospy.logwarn("error while port scanning (likely benighn): " + str(e))
	return("")


def listener():
	pass


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
	pass 


def spin_monitor_estop():
	print("spinning up nodes")
	rospy.init_node('estop')
	print("starting up threads")
	commander = threading.Thread(target = monitor)
	commander.start()
	print("nodes online")

def monitor():
	""" 
		This thread repeatedly polls the e-stop teensy to determine if 
		the transponder has requested an e-stop. If it has the e-stop topic
		is set to true. 
	"""
	pub = rospy.Publisher(ESTOP_TOPIC, Bool, queue_size = 0)
	rate = rospy.Rate(10) #set polling rate in hz
	failed_count = 0
	while not rospy.is_shutdown():
		rate.sleep()
		responce = check_tier_2(pub)


		if responce[0]:
			# if the transmission worked, reset the counter. 
			failed_count = 0
		else: 
			failed_count += 1 #incriment the failure counter. 
			#rospy.logwarn("steering update failed")
		if (failed_count > 0 and failed_count % ALLOWED_FAILURES == 0):
			print(scan_ports(DEVICE_ID))

def check_tier_2(pub):
	"""
		This function checks for the presence of a transponder e-stop, and 
		publishes "True" to the prvoided ros publisher if one is encountered.. 
		args:
			pub - a rospy publisher to use for indicating an error. 

		returns: unmodified return from read_pin
	"""
	responce = read_pin(PIN_TRANSPONDER)
	print (responce)
	(error, triggered) = responce
	if triggered and error:
		pub.publish(True)
	elif not error:
		rospy.logwarn("Errors: E-stop system error")

	return(responce)





def read_pin(pin):
	"""
		
		returns: a tuple of the form (bool, bool). The first is true when no errors
				 were encountered (detected). The second is true when the pin measured
				 read as high. 
	"""
	payload = bytearray.fromhex("0000")
	responce = send_frame(COMMAND_READ_D, payload, RESPONCE_READ_D_LN)
	responce_packet = responce[1]

	no_error_serial = responce[0] #true if no errors. 


	# note: index 3 will contain teensy errors if any. 
	print(responce_packet)

	# check no serial errors and correct responce length. 
	if no_error_serial and len(responce_packet) == RESPONCE_READ_D_LN:
		value = bool(responce_packet[RESPONCE_INDEX_DATA])
		ID = responce_packet[RESPONCE_INDEX_ID_S:RESPONCE_INDEX_ID_E]
		# check no teensy errors and correct device. 
		if not responce_packet[RESPONCE_INDEX_ERROR] and ID == DEVICE_ID:
			return(responce[0], value)
	
	return(False, 0)







def write_pin(pin, value):
	payload = bytearray.fromhex("0000")
	responce = send_frame(COMMAND_WRITE_D, payload, 3)
	responce_packet = responce[1]
	
	verification_packet = payload[0:1] + DEVICE_ID
	if (verification_packet != responce_packet):
		scan_ports(DEVICE_ID)

	global count
	count = 0
	return responce

if __name__ == '__main__':
	print("Scanning for ports")
	print(scan_ports(DEVICE_ID))
	time.sleep(1)
	
	spin_monitor_estop()
	listener()


