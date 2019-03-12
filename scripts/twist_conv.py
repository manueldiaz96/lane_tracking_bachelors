#!/usr/bin/env python
# coding=utf-8

import roslib
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from decimal import Decimal
import numpy as np
import sys

twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

def truncator(num):
	n_dec = 7
	limit_a = str(int(num)) + str(Decimal(str(num)) % 1)[1:n_dec] 

	return float(limit_a)


def joint_state_create(angle):

	js_msg = JointState()
	js_msg.header = Header(stamp=rospy.Time.now())
	js_msg.name = ['steering_joint', 'front_left_steer_joint', 'front_right_steer_joint', 'front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
  	js_msg.position = [angle, angle, angle, 0.0, 0.0, 0.0, 0.0]

  	return js_msg
	

def create_Twist(angle):

	tw_msg = Twist()
	tw_msg.linear.x = 0.5
	angle_twist = (4*angle)/np.pi
	angle_twist = truncator(angle_twist)
	tw_msg.angular.z=-1*angle_twist

	return tw_msg


def callback(msg):
	angle = msg.data
	
	twist_pub.publish(create_Twist(angle))
	joint_state_pub.publish(joint_state_create(-angle))


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