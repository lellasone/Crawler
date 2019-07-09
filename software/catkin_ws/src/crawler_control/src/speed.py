#!/usr/bin/env python3

# This module is responsible for the Crawler's speed control. It is run when 
# the robot is first booted. 
#
# Maintainer: Jake ketchum, jketchum@caltech.edu

import rospy
from std_msgs.msg import Float64
import odrive
import threading 
import time

MAX_RPM = 100 #max RPM of motor
CPR = 4000 # encoder counts per motor revolution. 
MAX_PPS = CPR * MAX_RPM #in pulses per revolution (hardware dependent)
MAX_CURRENT = 60 #in amps

engine = ''



def init_node():
	print("Setting up listener")
	rospy.init_node('throttle',anonymous=True)


def listener():
	rospy.Subscriber("/crawler/command_speed", Float64, callback)
	print("spinning up listener")
	rospy.spin()
	print("listener down")

def callback(msg):
	'''
		This function is called each time a new speed request is detected. 
	'''
	speed = msg.data
	speed = compute_pps(speed) #scale from to correct speeds.
	set_speed(int(speed))
	rospy.loginfo("speed_requested: " +str(speed))
	rospy.loginfo("Iq_measured: " + str(engine.axis0.motor.current_control.Iq_measured))

def set_speed(velocity):
	'''
		This function is responsible for setting the velocity setpoint paramiter. Note that 
		It functions in pulses per minute, and will thus provide a different velocity depending
		on the robot's hardware configuration. 

		note that if no odrive is connected this function will block until a new odrive is connected
		and the original speed will not be transmitted. 
		args: 
		- velocity: The desired robot velocity in pulses per second (should be an int)
	'''
	#TODO: add try-except statment. 
	if(check_living()):
		print(check_living())
		engine.axis1.controller.vel_setpoint = velocity
	else:
		rospy.logwarn("No Odrive Connected")
		setup_odrive()

def set_params():
	'''
		This function is sets various paramiters on the ODrive motor controller. 
		It should be called once, when the ODrive is initated, and subsequently
		every time the ODrive is re-booted. This allows ODrives to be swapped 
		between robots without requiring their settings be changed. 
	'''
	engine.axis1.motor.config.current_lim = MAX_CURRENT # Set maximum allowable current. 
	engine.axis1.controller.config.vel_limit = MAX_PPS + 20000 #add some buffer to prevent errors. 
	engine.axis1.motor.config.calibration_current = 20
	engine.config.brake_resistance = 0.8 
	engine.axis1.motor.config.pole_pairs = 2
	engine.axis1.encoder.config.cpr = CPR
	engine.axis1.motor.config.requested_current_range = MAX_CURRENT #set current sense gains. 
	engine.axis1.controller.config.control_mode = 2 #set to velocity control
	engine.axis1.encoder.config.use_index = True
	print("paramiters set")

def check_living():
	try: 
		engine.serial_number
	except: 
		return False
	return True

def compute_pps(rpm):
	'''
		This function takes the desired motor RPM and converts that to 
		encoder pulses per second. This is useful for determining what 
		speed to write to the odrive (which takes its speeds in pps)

		Note that in it's current form this function tends to be off by 
		around 0.5% depending on what the encoder CPR is calculated too
		on the odrive. 
		args:
			RPM - motor speed to be converted in rotations per minute. 

		returns: the pps as a float.
	'''
	rps = rpm / 60
	pps = rps * CPR 
	return(pps)


def startup_calibration():
	'''
		This function runs through the startup sequence for the robot. It should be called after
		all the desired odrive paramtiers have been set, but before any attempts are made to move
		the robot. Note, that this function takes a non-trivla time (order of 20 seconds) to execute. 
	'''
	print("calibrating motor and encoder")
	time.sleep(1)
	engine.axis1.requested_state = 4 
	time.sleep(4)
	engine.axis1.requested_state = 6
	time.sleep(5)
	engine.axis1.requested_state = 7
	time.sleep(5)
	engine.axis1.requested_state = 8
	print("motor and encoder calibration complete")

	#engine.save_configuration() #Not needed if called on startup. 

def setup_odrive():
	''' 
		this function is responsible for locating the odrive and creating the 
		Odrive object. It is important that this be called after the relavent 
		rosnode has been initated since error messages rely on roslog functionality. 
	'''
	global engine 
	print("finding odrive")
	engine = odrive.find_any()
	print("odrive found")
	print(engine.vbus_voltage)
	set_params()
	startup_calibration()
	print("odrive setup complete")


def reboot_odrive():
	''' 
		This function reboots the connected odrive and then finds it again.
		This is useful for debugging or as a last-ditch option for fixing
		errors when deployed. 

		This assumes the global engine exists and has already been populated.
	'''
	global engine 
	try:
		engine.reboot()
		time.sleep(10)
	except: 
		pass 

	engine = odrive.find_any()

def spin_analytics():
	'''
		This function is responsible for spinning up the threads that handle 
		status reporting for the o-drive.
	'''
	current = threading.Thread(target=broadcast_value, args = (engine.axis1.motor.current_control.Iq_measured,))
	current.start()

def broadcast_value(value):
	rate = rospy.Rate(50)
	while True: 
		rospy.loginfo(value)
		rospy.loginfo(engine.axis1.motor.current_control.Iq_measured)

		rate.sleep()



if __name__ == '__main__':
	init_node()
	setup_odrive()
	spin_analytics()
	listener()


