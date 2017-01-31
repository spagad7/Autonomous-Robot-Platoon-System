#!/usr/bin/env python

'''
Author:
=======
Shishir Pagad
Debraj Bhattacharjee

Date:
=====
Dec 12, 2016

Version:
========
10

Robot
=====
Pioneer 3DX

Laser Sensor
============
SICK LMS200

Python Version
==============
3.5.2

ROS Version
===========
ROS Kinetic

Description:
============
This code is for autonomously navigating leader robot in a hallway. The hallway is divided into 7 zones
and the goal of the leader robot is to stay at the center of the hallway and avoid any obstacle in its path
'''

import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class GoForward():
    	velocity = 0
    	movecmd = 0
	count = 30
	wallPref = 'left'

        # Initialize
    	def __init__(self):
		# Create a node in ROS
		rospy.init_node('GoForward', anonymous=False)
		rospy.loginfo("Leader Moving")
	  
		rospy.on_shutdown(self.shutdown)
		
		# Create Publisher to publish velocity commands,
		# and subscribe to scan topic on which laser scan data is published
		GoForward.velocity = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=10)
		self.laser = rospy.Subscriber('/scan', LaserScan, self.callback, queue_size=10)
	
		r = rospy.Rate(10);

		# Create movecmd message 
		GoForward.movecmd = Twist()
		GoForward.movecmd.linear.x = 0
		GoForward.movecmd.angular.z = 0
	
		while not rospy.is_shutdown():
		    # publish the velocity
			GoForward.velocity.publish(GoForward.movecmd)
			r.sleep()
		

	# Function for obstacle avoidance
	# This function takes laser scan data as input
	def obstacleAvoid(self, laserRange):
                # Stop upon detecting obstacle
		GoForward.movecmd.linear.x = 0.0
		GoForward.movecmd.angular.z = 0.0
		GoForward.velocity.publish(GoForward.movecmd)
		
		# Divide laser scan data in the range of [60:120] degree
		# into 3 sectors of 20 degrees each
		lr = []
		lr.append(min(laserRange[60:80]))
		lr.append(min(laserRange[80:100]))
		lr.append(min(laserRange[100:120]))

		# Find the sector with smallest distance
		# The sectorw with smallest distance has the obstacle
		tmp = lr[0]
		mrIndex = 0
		for i in range(3):
			if lr[i]<tmp:			
				tmp = lr[i]
				mrIndex = i
		# mrIndex has index of the segment where the object lies
		maxRange = lr[mrIndex]
		print('Object is in sector %d' %mrIndex)
		pr=1

		# move away from the obstacle based on the sector in which the obstacle lies
		if (mrIndex == 0):
			GoForward.movecmd.linear.x = 0.0
			GoForward.movecmd.angular.z = 0.1
			GoForward.velocity.publish(GoForward.movecmd)
		elif (mrIndex == 1):
			if lr[0]>lr[2]:
				pr=-1
			else:
				pr=1;
			GoForward.movecmd.linear.x = 0.0
			GoForward.movecmd.angular.z = 0.05*pr
			GoForward.velocity.publish(GoForward.movecmd)
		elif (mrIndex == 2):
			GoForward.movecmd.linear.x = 0.0
			GoForward.movecmd.angular.z = -0.1
			GoForward.velocity.publish(GoForward.movecmd)
				
		

        # This function is called every time when there is new laser scan data.
        # The leader robot follows the wall set in 'wallPref' variable. If it
        # is following left wall, then it uses 150 degree laser ray to measure
        # distance from left wall, and if it is following right wall then it
        # uses 30 degree ray to measure distance from right wall. 
    	def callback(self, laserData):
		laserRange=laserData.ranges

		# Code for switching walls
		if GoForward.wallPref == 'left':
			angle = 150
		elif GoForward.wallPref == 'right':
			angle = 30

		# Change sign of angular velocity based on wallPref
		if(angle == 150):
			direct = 1
		elif(angle == 30):
			direct = -1

		# Obstacle detection
                # The leader robot detects obstacle if it lies in the range of
                # 60 to 120 degree and less than 1.2 meter distance.
		obstacle = False
		midSector = laserRange[60:120]
		for i in range(60):
			if midSector[i] < 1.2:
				obstacle = True
				break
                # If obstacle is detected, call obstacle avoidance function
		if obstacle:
			GoForward.obstacleAvoid(self,laserRange)
		
		#Code for driving the robot
		#Divide hallway into 7 zones |zone1/nearleftwall|zone2|zone3|zone4/center|zone5|zone6|zone7/nearrightwall|
		#If in zone1, take sharp right turn
		elif(laserRange[angle]<0.53):
			print('In zone 1')
			GoForward.movecmd.linear.x = 0.2
			GoForward.movecmd.angular.z = -0.15*direct
			GoForward.velocity.publish(GoForward.movecmd)
		#If in zone2, take moderate right turn
		elif(laserRange[angle]>0.53 and laserRange[angle]<0.87):
			print('In zone 2')
			GoForward.movecmd.linear.x = 0.2
			GoForward.movecmd.angular.z = -0.10*direct
			GoForward.velocity.publish(GoForward.movecmd)
		#If in zone3, take light right turn to center zone
		elif(laserRange[angle]>0.87 and laserRange[angle]<1.215):
			print('In zone 3')
			GoForward.movecmd.linear.x = 0.2
			GoForward.movecmd.angular.z = -0.05*direct
			GoForward.velocity.publish(GoForward.movecmd)
		#If in zone4/center, go straight
		elif(laserRange[angle]>1.215 and laserRange[angle]<1.56):
			print('In center zone')
			GoForward.movecmd.linear.x = 0.2
			GoForward.movecmd.angular.z = 0.00
			GoForward.velocity.publish(GoForward.movecmd)
		#If in zone5, take light left turn to center zone
		elif(laserRange[angle]>1.56 and laserRange[angle]<1.913):
			print('In zone 5')
			GoForward.movecmd.linear.x = 0.2
			GoForward.movecmd.angular.z = 0.05*direct
			GoForward.velocity.publish(GoForward.movecmd)
		#If in zone6, take moderate left turn
		elif(laserRange[angle]>1.913 and laserRange[angle]<2.28):
			print('In zone 6')
			GoForward.movecmd.linear.x = 0.2
			GoForward.movecmd.angular.z = 0.10*direct
			GoForward.velocity.publish(GoForward.movecmd)
		#If in zone7, take sharp left turn
		elif(laserRange[angle]>2.28):
			print('In zone 7')
			GoForward.movecmd.linear.x = 0.2
			GoForward.movecmd.angular.z = 0.15*direct
			GoForward.velocity.publish(GoForward.movecmd)
		
                  
	def shutdown(self):
		# stop leader robot
		rospy.loginfo("Leader Stopped")
		GoForward.velocity.publish(Twist())
		rospy.sleep(1)
    

if __name__ == '__main__':
    try:
	GoForward()
     
    except:
        rospy.loginfo("GoForward node terminated.")

	           
       
