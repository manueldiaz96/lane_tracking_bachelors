#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.image_pub = rospy.Publisher("/line_detector/image_lines",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    lines = rospy.get_param('/detected_lines')

    cv_image = self.detect_lines(cv_image, lines)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def detect_lines(self, cv_image, lines):
    #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
    (h,w,_) = cv_image.shape

    h_s = (h/12)*5

    for line in lines:
      for x1, y1, x2, y2 in line:

        cv2.line(cv_image, (x1,y1+h_s), (x2,y2+h_s), [0,200,0], 3)

    return cv_image

def main(args):
  rospy.init_node('line_detector_2', anonymous=True)
  rospy.loginfo("Line detector on")
  ic = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  

if __name__ == '__main__':
    main(sys.argv)