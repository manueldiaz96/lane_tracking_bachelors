#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from sklearn.preprocessing import MinMaxScaler
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
      gradient_x = cv2.Sobel(gradient, cv2.CV_32F, 1, 0, ksize=1)
      gradient_y = cv2.Sobel(gradient, cv2.CV_32F, 0, 1, ksize=1)

      mag, angle = cv2.cartToPolar(gradient_x, gradient_y, angleInDegrees=True)

      im_gradient_x = self.scaleTo(gradient_x, max_val=255)
      im_gradient_y = self.scaleTo(gradient_y, max_val=255)
      im_mag = self.scaleTo(mag, max_val=255)
      im_angle = self.scaleTo(angle, max_val=255)

      for line in lines:

        for x1, y1, x2, y2 in line:

          line_img = np.hstack((im_gradient_x[y1-20:y2+20, x1-20:x2+20], im_gradient_y[y1-20:y2+20, x1-20:x2+20], im_mag[y1-20:y2+20, x1-20:x2+20], im_angle[y1-20:y2+20, x1-20:x2+20], gray[y1-20:y2+20, x1-20:x2+20]))

          if line_img.shape[0] and line_img.shape[1]:
            histogram = self.get_histogram(mag[y1-20:y2+20, x1-20:x2+20], angle[y1-20:y2+20, x1-20:x2+20], hist_type = "ang2mag")
            print(histogram)
            cv2.imwrite("/home/manuel/Pictures/dataset_lines/frame_%d_line_%d.jpg" % (count_frame,count_img), line_img)

        count_img = count_img +1

    count_frame = count_frame+1
    rospy.set_param('/frame', count_frame)


  def scaleTo(self, data, max_val):
    data = data.astype(np.float)

    scaler = MinMaxScaler()
    scaler.fit(data)
    data = scaler.transform(data)

    data = (data*max_val).astype(np.int)

    return data


  def get_histogram (self, mag, angle, hist_type):
    #Binning of 10

    if hist_type == "ang2ang":
      #Angle respect to angle
      hist = np.zeros((36,))
      for f in range(0, angle.shape[0]):
        for c in range(0, angle.shape[1]):

          i = (angle[f,c]/10).astype(np.int)
          #print (i)
          if i == 36:
            i = 0

          hist[i] += 1


    elif hist_type == "mag2mag":
      #Magnitude respect to magnitude
      hist = np.zeros((10,))

      mag = self.scaleTo(mag, max_val=100)

      for f in range(0, angle.shape[0]):
        for c in range(0, angle.shape[1]):

          i = (mag[f,c]/10).astype(np.int)

          if i == 10:
            i = 9

          hist[i] += 1


    elif hist_type == "ang2mag":
      #Angle respect to magnitude
      hist = np.zeros((36,))

      mag = self.scaleTo(mag, max_val=100)

      for f in range(0, angle.shape[0]):
        for c in range(0, angle.shape[1]):

          ang_i = (angle[f,c]/10).astype(np.int)

          if ang_i == 36:
            ang_i = 0

          mag_i = (mag[f,c]/10).astype(np.int)

          hist[ang_i] += mag_i


    return hist.astype(np.int)



def main(args):
  rospy.init_node('line_detector', anonymous=True)
  rospy.loginfo("Data preprocessing on")
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.set_param('/frame', 0)
    rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
