#!/usr/bin/env python3

#==============================================================================
# File name          : exe1.py                                                                 
# Description        : Exe1 for CS588, jingyul9 & yutaoz3                                                                                                                        
# Usage              : rosrun exe1 exe1.py                                                                                                                           
#==============================================================================
from __future__ import print_function

#Python Headers
import math
import os

# ROS Headers
import rospy
import time

# GEM PACMod Headers
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt

class Node():

	def __init__(self):

		# self.rate = rospy.Rate(1.42)
		self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size = 1)
		self.turn_cmd = PacmodCmd()

	def run(self):
		count = 0
		light = [2, 2, 2, 0, 0, 0, 2, 2, 2]
		while not rospy.is_shutdown():
			self.turn_cmd.ui16_cmd = light[count]
			self.turn_pub.publish(self.turn_cmd)
			count += 1
			# self.rate.sleep()
			time.sleep(0.7)  # Match the frequency of the light
			if (count > 9):
				count = 0
				self.turn_cmd.ui16_cmd = 1  # Switch off the light
				self.turn_cmd.publish(self.turn_cmd)
				time.sleep(2.1) # Pause

if __name__ == '__main__':
	rospy.init_node('sos_node', anonymous=True)
	node = Node()
	node.run()








	