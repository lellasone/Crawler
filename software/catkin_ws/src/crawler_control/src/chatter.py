#!/usr/bin/env python

import rospy
from std_msgs.msg import String 

pub = rospy.Publisher('chatter', String, queue_size = 10)
rospy.init_node('talker', anonymous = True)

rate = rospy.Rate(50)

while not rospy.is_shutdown(): 
	pub.publish("JPL")
	rate.sleep()