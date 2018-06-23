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
    #self.image_pub = rospy.Publisher("/line_detector/image_lines",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image = self.classify_lines(cv_image)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

  def classify_lines(self, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 60, 80)

    hough_img = canny
    rows, cols = hough_img.shape

    theta = np.pi/180.0
    rho = 13
    min_line_len = 30 #The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    max_line_gap = 5 #The maximum gap between two points to be considered in the same line.
    threshold = 45 #The minimum number of intersections to “detect” a line

    lines = cv2.HoughLinesP(hough_img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    count_img = 0
    count_frame = rospy.get_param('/frame')

    if (count_frame%60) == 0:
      gray_rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

      gradient = np.float32(gray) / 255.0
      gradient_x = cv2.cvtColor(cv2.Sobel(gradient, cv2.CV_32F, 1, 0, ksize=1), cv2.COLOR_GRAY2BGR)
      gradient_y = cv2.cvtColor(cv2.Sobel(gradient, cv2.CV_32F, 0, 1, ksize=1), cv2.COLOR_GRAY2BGR)


      for line in lines:
       
        for x1, y1, x2, y2 in line:
          lines_rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
          cv2.line(lines_rgb, (x1, y1), (x2, y2), [0, 200, 0], 2)

          line_img = np.hstack((gradient_x[y1-20:y2+20, x1-20:x2+20, :], gradient_y[y1-20:y2+20, x1-20:x2+20, :] ,lines_rgb[y1-20:y2+20, x1-20:x2+20, :]))

          if line_img.shape[0] and line_img.shape[0]:
            cv2.imwrite("/home/manuel/Pictures/dataset_lines/frame_%d_line_%d.jpg" % (count_frame,count_img), line_img)

        count_img = count_img +1

    count_frame = count_frame+1
    rospy.set_param('/frame', count_frame)



def main(args):
  rospy.init_node('line_detector', anonymous=True)
  rospy.loginfo("Line detector on")
  ic = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.set_param('/frame', 0)
    rospy.loginfo("Shutting down")
  

if __name__ == '__main__':
    main(sys.argv)