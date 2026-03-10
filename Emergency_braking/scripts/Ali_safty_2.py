#!/usr/bin/env python
import rospy
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

# TODO: import ROS msg types and libraries

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
 
        self.pub_brake_bool = rospy.Publisher('/brake_bool', Bool, queue_size=10)
	self.pub_brake = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
	self.current_speed=0.0
    	rospy.Subscriber('/scan', LaserScan, self.scan_callback)
    	rospy.Subscriber('/odom', Odometry, self.odom_callback)
   	



        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        
        # TODO: create ROS subscribers and publishers.

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.current_speed = math.sqrt (odom_msg.twist.twist.linear.x**2 + odom_msg.twist.twist.linear.y**2)
	

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC

        # TODO: publish brake message and publish controller bool

  	min_TTC=0.7
    	for i in range (len(scan_msg.ranges)):
	    r_dot= self.current_speed * math.cos((scan_msg.angle_min) + (scan_msg.angle_increment*i))
	    TTC=scan_msg.ranges[i]/(max(-r_dot,0.000000001))

	    if  TTC<min_TTC:
	        self.pub_brake_bool.publish(True)
	        a=AckermannDriveStamped()
	        a.drive.speed=0.0
   	        self.pub_brake.publish(a)
		print ('brake now')
		print TTC
	        break
	   

	#print len(scan_msg.ranges)


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()

