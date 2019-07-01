#!/usr/bin/env python3

# This module is responsible for the Crawler's speed control. It is run when 
# the robot is first booted. 
#
# Maintainer: Jake ketchum, jketchum@caltech.edu

import rospy
from sensor_msgs.msg import Joy
import odrive

INDEX_PAN = 0
INDEX_SPEED = 1

MAX_SPEED = 200000 #in pulses per revolution (hardware dependent)
MAX_CURRENT = 80 #in amps

def init_node():
	print("Setting up listener")
	rospy.init_node('throttle',anonymous=True)


def listener():
	rospy.Subscriber("/spacenav/joy", Joy, callback)
	print("spinning up listener")
	rospy.spin()
	print("listener down")

def callback(msg):
	speed = msg.axes[INDEX_SPEED]
	speed = speed * -MAX_SPEED #scale from to correct speeds.
	set_speed(int(speed))
	rospy.loginfo("speed_requested: " +str(speed))
	rospy.loginfo("Iq_measured: " + str(engine.axis0.motor.current_control.Iq_measured))

def set_speed(speed):
	#TODO: add try-except statment. 
	engine.axis0.controller.vel_setpoint = speed


def set_params():
	engine.axis0.motor.config.current_lim = MAX_CURRENT # Set maximum allowable current. 
	engine.axis0.controller.config.vel_limit = MAX_SPEED + 50000 #add some buffer to prevent errors. 
	engine.axis0.motor.config.calibration_current = 20
	engine.config.brake_resistance = 0.8 
	engine.axis0.motor.config.pole_pairs = 2
	engine.axis0.encoder.config.cpr = 2880
	engine.axis0.motor.config.requested_current_range = MAX_CURRENT #set current sense gains. 
	#engine.save_configuration() #Not needed if called on startup. 

def setup_odrive():
	global engine 
	print("finding odrive")
	engine = odrive.find_any()
	print("odrive found")
	print(engine.vbus_voltage)
	set_params()

if __name__ == '__main__':
	init_node()
	setup_odrive()
	listener()


