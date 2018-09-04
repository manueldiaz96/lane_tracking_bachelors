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
    self.image_sub = rospy.Subscriber("/Haar_edges/image",Image,self.callback)
    self.image_pub = rospy.Publisher("/line_detector/image_lines",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    cv_image= self.detect_lines(cv_image)

    is_bgr = len(cv_image.shape) == 3

    if True:
      try:
        if is_bgr:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        else:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
      except CvBridgeError as e:
        print(e)



  def detect_lines(self, edges):

    img = np.dstack((edges, edges, edges))


    h, w = edges.shape 

    theta = np.pi/180.0
    rho = 13 #Distance resolution of the accumulator in pixels. 
    min_line_len = 20 #The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    max_line_gap = 10 #The maximum gap between two points to be considered in the same line.
    threshold = 15 #The minimum number of intersections to “detect” a line

    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    horizon_line = rospy.get_param('/horizonLine')
    margin = rospy.get_param('/margin')

    leftLinek1 = rospy.get_param('/leftLinek1')
    leftLinek2 = rospy.get_param('/leftLinek2')
    leftLinek3 = rospy.get_param('/leftLinek3')

    rightLinek1 = rospy.get_param('/rightLinek1')
    rightLinek2 = rospy.get_param('/rightLinek2')
    rightLinek3 = rospy.get_param('/rightLinek3')

    right_slopes = []
    left_slopes = []
    right_int = []
    left_int = []

    x1_l = []
    x2_l = []
    y1_l = []
    y2_l = []

    x1_r = []
    x2_r = []
    y1_r = []
    y2_r = []


    m_right = 0
    m_left = 0
    b_right = 0
    b_left = 0


    if len(lines):
      for line in lines:
        for x1,y1,x2,y2 in line:
          if ((x2-x1) != 0):
            slope = ((float(y2)-float(y1))/(float(x2)-float(x1)))
            intercept = y1 - (slope*x1)

            is_right = (slope > 0.6) and (slope < 0.8) and (x1 > (w/2)+margin) and (x2 > (w/2)+margin) 
            is_left = (slope < -0.6) and (slope > -0.8) and (x2 < (w/2)+margin) and (x2 < (w/2)+margin)          

            if not np.mean(horizon_line):
              is_below_horizon = (y1 > h/2) and (y2 > h/2)

            else:
              is_below_horizon = (y1 > horizon_line[0]) and (y2 > horizon_line[0])

            if is_below_horizon:

              if is_left:
                x1_l.append(x1)
                x2_l.append(x2)
                y1_l.append(y1)
                y2_l.append(y2)
                #cv2.line(img, (x1, y1), (x2, y2), [200, 0, 0], 3)

              elif is_right:
                x1_r.append(x1)
                x2_r.append(x2)
                y1_r.append(y1)
                y2_r.append(y2)
                #cv2.line(img, (x1, y1), (x2, y2), [0, 200, 0], 3)

      if len(x1_r):
        rightLine = [np.int(np.mean(x1_r)), np.int(np.mean(y1_r)), np.int(np.mean(x2_r)), np.int(np.mean(y2_r))]
        x1, y1, x2, y2 = rightLine
        mr = float(y2-y1)/float(x2-x1)
        br = y1 - mr*x1
        rospy.set_param('/rightLine', [mr, br])

      else:
        mr, br = rospy.get_param('/rightLine')



      if len(x1_l):
        leftLine = [np.int(np.mean(x1_l)), np.int(np.mean(y1_l)), np.int(np.mean(x2_l)), np.int(np.mean(y2_l))]
        x1, y1, x2, y2 = leftLine
        ml = float(y2-y1)/float(x2-x1)
        bl = y1 - ml*x1
        rospy.set_param('/leftLine', [ml ,bl])

      else:
        ml, bl = rospy.get_param('/leftLine')


      if (ml!=0) and (mr!=0):
        x_horizon = (br - bl)/(ml - mr)
        y_horizon = ml*x_horizon + bl

        xl_bottom = np.int((h-bl)/ml)-100
        xr_bottom = np.int((h-br)/mr)+100

        cv2.line(img, (int(x_horizon)-100, int(y_horizon)), (int(x_horizon)+100, int(y_horizon)), [0, 0, 123], 3)
        cv2.line(img, (int(x_horizon)-100, int(y_horizon)), (xl_bottom, h-1), [0, 0, 123], 3)
        cv2.line(img, (int(x_horizon)+100, int(y_horizon)), (xr_bottom, h-1), [0, 0, 123], 3)

        
    


    #   if len(right_slopes):
    #     m_right = np.mean(right_slopes)
    #     b_right = np.mean(right_int)

    #   if len(left_slopes):
    #     m_left = np.mean(left_slopes)
    #     b_left = np.mean(left_int)

    #   if m_left and m_right:
    #     x_horizon = (b_right - b_left)/(m_left - m_right)
    #     y_horizon = m_left*x_horizon + b_left

    #     if y_horizon < 0:
    #       y_horizon = 0

    #     horizon_line = [int(y_horizon), int(horizon_line[0]), int(horizon_line[1])]
    #     horizon_line[0] = int(np.mean(horizon_line))
    #     y_horizon = horizon_line[0]
    #     print(horizon_line)
    #     rospy.set_param('/horizonLine', horizon_line)

    #     x_right_bottom = (h - b_right) / (m_right)
    #     x_left_bottom = (h - b_left) / (m_left)

    #     leftLine = [ int(x_horizon), int(y_horizon), int(x_left_bottom), int(h-1) ]
    #     rightLine = [ int(x_horizon), int(y_horizon), int(x_right_bottom), int(h-1) ]

    #     cv2.line(img, (leftLine[0], leftLine[1]), (leftLine[2], leftLine[3]), [0, 200, 0], 3)
    #     cv2.line(img, (rightLine[0], rightLine[1]), (rightLine[2], rightLine[3]), [0, 200, 0], 3)
    #     cv2.line(img, (0, int(y_horizon)), (w-1, int(y_horizon)), [0, 0, 123], 3)

    # if lines == None:
    #   print("Holi")

          

    return img

def main(args):
  rospy.init_node('line_detector', anonymous=True)
  rospy.loginfo("Line detector on")
  ic = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  

if __name__ == '__main__':
    main(sys.argv)