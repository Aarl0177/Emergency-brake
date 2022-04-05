#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 0.01 #TODO
kd = 0.1 #TODO
ki = 0.01 #TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
vel_input = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.getRange) #TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size=10) #TODO: Publish to drive

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
	for i in range (len(scan_msg.ranges)):
	    angle= scan_msg.angle_min + (scan_msg.angle_increment*i)
	    if  angle[i]==0.0:
	    	b=scan_msg.ranges[i]
	    elif angle[i]==50.0:
		 a=scan_msg.ranges[i]
		 
	    	 break
	    alpha=math.atan((a*math.cos(angle[i])-b)/(a*math.sin(angle[i}))
	    Dt=b*math.cos(alpha)
	    Dt1=Dt+CAR_LENGTH *math.sin(alpha)
	    error=DESIRED_DISTANCE_RIGHT-Dt1
	    

    def pid_control(self, error, velocity):
        global integral
        global prev_error
	global vel_input
        global kp
        global ki
        global kd
        angle = 0.0
	angle=kp*error+ki*integral-kd*vel_input
	if angle=>0.0 and angle<=10.0:
	   velocity=1.5
	elif angle=>10.0 and angle<=20.0:
	     velocity=1.0
	else 
	     velocity=0.5
        #TODO: Use kp, ki & kd to implement a PID controller for 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        return 0.0 

    def lidar_callback(self, data):
        """ 
        """
        error = 0.0 #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)

