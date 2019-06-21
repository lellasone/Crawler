#!/usr/bin/env python

# This is a proof of concept for teensy based steering.

import rospy
from sensor_msgs.msg import Joy

INDEX_PAN = 3
INDEX_SPEED = 4

COMMAND_STEERING = bytearray.fromhex("62")

def listener():
	print("Setting up listener")
	rospy.init_node('listener',anonymous=True)

	rospy.Subscriber("/spacenav/joy", Joy, callback)
	
	print("spinning up listener")
	rospy.spin()
	print("listener down")

def callback(msg):
	angle = msg.axes[INDEX_PAN]
	angle = angle + 1 #re-center as positive
	angle = angle * 128 # scale to 8 bits. 
	set_steering(int(angle))

def set_steering(direction):
        if direction > 0 and direction < 256:

                message = bytes([direction]) + bytearray.fromhex("0000")
                send_frame(COMMAND_STEERING, message)
        else: 
                print("invalid direction request: " + str(direction))

def send_frame(command, data):
        with open("/dev/ttyACM0", 'wb') as teensy:
                frame = bytearray.fromhex("41")
                frame = frame + command + data
                frame = frame + bytearray.fromhex("5A") 
                print(frame)
                teensy.write(frame)
                print("complete")



if __name__ == '__main__':
	
	listener()


