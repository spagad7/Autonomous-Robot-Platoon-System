#!/usr/bin/env python

'''
Author:
=======
Aniket Pendse
Debraj Bhattacharjee
Amol Vagad
Shishir Pagad

Date:
=====
Dec 12, 2016

Version:
========
7

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
This code is for follower robot, as the name suggests, it autonomously follows a leader robot.
A follower robot uses segmentation to detect the leader robot infront of it and calculates the
angular and linear velocity based on the angle and distance between them. 
'''

import rospy
import math
from numpy import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class GoForward():
    velocity = 0
    movecmd = 0

    # Initialize
    def __init__(self):
        # Create a node in ROS
	rospy.init_node('GoForward', anonymous=False)
	rospy.loginfo("Follower Started")
  
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

    # This function is called every time when there is new laser scan data.
    # The follower robot uses segmentation to detect the leader robot in front of it.
    # After detecting the leader, the follower uses circle fitting to follow the follower robot.
    def callback(self, laserData):
	self.laserRange=laserData.ranges
	self.diff = list()
	self.segment = list()
	self.diff.append(0)

	# Create segments based on change in distance beyond a certain threshold between adjacent laser rays
	for i in range(179):
		if (self.laserRange[i+1] - self.laserRange[i]) < -0.5:
			self.diff.append(i+1)
		elif (self.laserRange[i+1] - self.laserRange[i]) > 0.5:
			self.diff.append(i)
	self.diff.append(179)
	k=0

	# Find Cartesian coordinates of the segments from polar coordinates
	# (laser scan data is in polar coordinates)
	for i in range(len(self.diff)-1):
		self.xCoord1 = self.laserRange[self.diff[i]]*math.cos((self.diff[i]*math.pi)/180)
		self.yCoord1 = self.laserRange[self.diff[i]]*math.sin((self.diff[i]*math.pi)/180)
		self.xCoord2 = self.laserRange[self.diff[i+1]]*math.cos((self.diff[i+1]*math.pi)/180)
		self.yCoord2 = self.laserRange[self.diff[i+1]]*math.sin((self.diff[i+1]*math.pi)/180)
		# Find the width of the object
		self.dist = math.sqrt((self.xCoord1-self.xCoord2)**2 + (self.yCoord1-self.yCoord2)**2)

		# Check if the width of object is between 0.3m to 0.4m (Pioneer 3DX with laptop on it
		# has width of approximately 0.35m)
		if self.dist>0.3 and self.dist<0.4:
			x1 = self.xCoord1
			x2 = self.xCoord2
			y1 = self.yCoord1
			y2 = self.yCoord2
			break
		else:
			x1 = 0
			x2 = 0
			y1 = 0
			y2 = 0
		print ("Distance (%d,%d) = %f" %(self.diff[i], self.diff[i+1], self.dist))


        # Circle fitting method
	print("(x1,y1),(x2,y2)=(%f,%f),(%f,%f)" %(x1,y1,x2,y2))
	
	# Find mid point of the leader robot
	x3=(x1+x2)/2
	y3=(y1+y2)/2
	obstacle = math.sqrt((x3**2)+(y3**2))
	print("Obstcle is %f far"%obstacle)
	print("(x3,y3)=(%f,%f)"%(x3,y3))
	
        if (x3**2+y3**2)>=0.5:
	    print("Leader atleast 0.5m away")
	    # Calculate angle between follower and leader
	    t1 = math.atan((x1-x2)/(y2-y1))
	    
	    # Calculate angular and linear velocity based on the angle and
	    # distance between leader and follower robot
	    if t1>=0:
		w = (t1-(math.pi/2))/5
		r = obstacle/math.sin(abs(t1-(math.pi/2)))/2
		v = r*abs(t1-(math.pi/2))/5
	    elif t1<0:
		w = ((math.pi/2)+t1)/5
		r = obstacle/math.sin((math.pi/2)+t1)/2
		v = r*((math.pi/2)+t1)/5
		
	    print("Angular Velocity = %f" %w)
	    print("Linear Velocity = %f"%v)
	    
	    # Publish velocity
	    GoForward.movecmd.linear.x = v
	    GoForward.movecmd.angular.z = w
	    GoForward.velocity.publish(GoForward.movecmd)
	    
	else:
	    print("Leader too close")
	    GoForward.movecmd.linear.x = 0.0
	    GoForward.movecmd.angular.z = 0
	    GoForward.velocity.publish(GoForward.movecmd)
	    

    def shutdown(self):
	# stop follower robot
	rospy.loginfo("Follower Stopped")
	GoForward.velocity.publish(Twist())
	rospy.sleep(1)
	
 
if __name__ == '__main__':
    try:
	GoForward()
     
    except:
        rospy.loginfo("GoForward node terminated.")
