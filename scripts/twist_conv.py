#!/usr/bin/env python
# coding=utf-8

import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from decimal import Decimal
import numpy as np
import sys

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

def truncator(num):
	n_dec = 7
	limit_a = str(int(num)) + str(Decimal(str(num)) % 1)[1:n_dec] 
	return float(limit_a)

def callback(msg):

	tw_msg = Twist()
	tw_msg.linear.x = 0.5
	angle = msg.data
	angle_twist = (4*angle)/np.pi
	angle_twist = truncator(angle_twist)
	tw_msg.angular.z=-1*angle_twist
	pub.publish(tw_msg)



def main(args):
	rospy.init_node('angle_listener', anonymous=True)
	rospy.loginfo("Angle Listener on")
	sub = rospy.Subscriber("/steer_angle_img", Float64, callback)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")


if __name__ == '__main__':
	main(sys.argv)