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
    self.image_sub = rospy.Subscriber("/edge_detector/image_edges",Image,self.callback)
    self.image_pub = rospy.Publisher("/line_detector/image_lines",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    lines = rospy.get_param('/detected_lines')

    cv_image = self.detect_lines(cv_image, lines)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)



  def detect_lines(self, img, lines):

    rows, cols = img.shape
    
    right_slopes = []
    left_slopes = []
    right_int = []
    left_int = []

    horizon_line = rospy.get_param('/horizonLine')

    leftLinek1 = rospy.get_param('/leftLinek1')
    leftLinek2 = rospy.get_param('/leftLinek2')
    leftLinek3 = rospy.get_param('/leftLinek3')

    rightLinek1 = rospy.get_param('/rightLinek1')
    rightLinek2 = rospy.get_param('/rightLinek2')
    rightLinek3 = rospy.get_param('/rightLinek3')

    for line in lines:

      for x1, y1, x2, y2 in line:

        if ((x2-x1) != 0): 

          slope = ((float(y2)-float(y1))/(float(x2)-float(x1)))
          intercept = y1 - (slope*x1)

          is_right = (slope > 0.3) and (x1 > cols/2) and (x2 > cols/2) 
          is_left = (slope < -0.3) and (x2 < cols/2) and (x2 < cols/2) 

          if not np.mean(horizon_line):
            is_above_horizon = (y1 < rows/2) and (y2 < rows/2)
            is_below_horizon = (y1 > rows/2) and (y2 > rows/2)

          else:
            is_above_horizon = (y1 < horizon_line[0]) and (y2 < horizon_line[0])
            is_below_horizon = (y1 > horizon_line[0]) and (y2 > horizon_line[0])

          if is_below_horizon:

            if is_left:
              left_slopes.append(slope)
              left_int.append(intercept)

            elif is_right:
              right_slopes.append(slope)
              right_int.append(intercept)


    if not left_slopes:
      if (float(leftLinek1[2])-float(leftLinek1[0])) == 0:
        leftLinek1[2] = 1+leftLinek1[0]
      m_left = ((float(leftLinek1[3])-float(leftLinek1[1]))/(float(leftLinek1[2])-float(leftLinek1[0])))
      b_left = leftLinek1[1] - (m_left*leftLinek1[0])

    else:
      m_left = np.mean(left_slopes)
      b_left = np.mean(left_int)

    if not right_slopes:
      if (float(rightLinek1[2])-float(rightLinek1[0])) == 0:
        rightLinek1[2] = 1+rightLinek1[0]
      m_right = ((float(rightLinek1[3])-float(rightLinek1[1]))/(float(rightLinek1[2])-float(rightLinek1[0])))
      b_right = rightLinek1[1] - (m_right*rightLinek1[0])

    else:
      m_right = np.mean(right_slopes)
      b_right = np.mean(right_int)

    if (m_left- m_right) == 0:
      m_left = 1+m_right

    x_horizon = (b_right - b_left)/(m_left - m_right)
    y_horizon = m_left*x_horizon + b_left

    horizon_line[2] = int(horizon_line[1])
    horizon_line[1] = int(horizon_line[0])
    horizon_line[0] = int(y_horizon)
    horizon_line[0] = int(np.mean(horizon_line))
    rospy.set_param('/horizonLine', horizon_line)

    x_right_bottom = (rows - b_right) / (m_right+1)
    x_left_bottom = (rows - b_left) / (m_left+1)

    leftLine = [ int(x_horizon), int(y_horizon), int(x_left_bottom), int(rows-1) ]
    rightLine = [ int(x_horizon), int(y_horizon), int(x_right_bottom), int(rows-1) ]

    for i in range(len(leftLine)):
      
      leftLinek3[i] = leftLinek2[i]
      leftLinek2[i] = leftLinek1[i]
      leftLinek1[i] = leftLine[i]
      leftLinek1[i] = (leftLinek1[i] + leftLinek2[i] + leftLinek3[i])/3 
      
      rightLinek3[i] = rightLinek2[i]
      rightLinek2[i] = rightLinek1[i]
      rightLinek1[i] = rightLine[i]
      rightLinek1[i] = (rightLinek1[i] + rightLinek2[i] + rightLinek3[i])/3 

    rightLine = rightLinek1
    leftLine = leftLinek1

    rospy.set_param('/leftLinek1', leftLinek1)
    rospy.set_param('/leftLinek2', leftLinek2)
    rospy.set_param('/leftLinek3', leftLinek3)

    rospy.set_param('/rightLinek1', rightLinek1)
    rospy.set_param('/rightLinek2', rightLinek2)
    rospy.set_param('/rightLinek3', rightLinek3)

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    cv2.line(img, (leftLine[0], leftLine[1]), (leftLine[2], leftLine[3]), [0, 200, 0], 3)
    cv2.line(img, (rightLine[0], rightLine[1]), (rightLine[2], rightLine[3]), [0, 200, 0], 3)
    cv2.line(img, (0, int(y_horizon)), (cols-1, int(y_horizon)), [0, 0, 123], 3)

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