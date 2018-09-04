#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function

import roslib
import sys
import rospy

import cv2
import numpy as np

from sklearn.preprocessing import MinMaxScaler
from sklearn.externals import joblib
#from sklearn.svm import SVC
from sklearn.neural_network import MLPClassifier

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.image_pub = rospy.Publisher("/edge_detector/image_edges",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image = self.detect_edges(cv_image)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
    except CvBridgeError as e:
      print(e)

  def detect_edges(self, img):   
    (h,w,_) = img.shape

    h_s = (h/12)

    img = img[h_s*5:h-h_s,:]

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 60, 80)

    lines = self.line_classifier(gray, canny)

    #img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    
    return canny

  def line_classifier(self, gray, canny):

    rows, cols = canny.shape

    theta = np.pi/180.0
    rho = 13
    min_line_len = 30 #The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    max_line_gap = 5 #The maximum gap between two points to be considered in the same line.
    threshold = 45 #The minimum number of intersections to “detect” a line

    lines_h = cv2.HoughLinesP(canny, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    gradient = np.float32(gray) / 255.0
    gradient_x = cv2.Sobel(gradient, cv2.CV_32F, 1, 0, ksize=1)
    gradient_y = cv2.Sobel(gradient, cv2.CV_32F, 0, 1, ksize=1)
    mag, angle = cv2.cartToPolar(gradient_x, gradient_y, angleInDegrees=True)
    mag = self.scaleTo(mag, max_val=100)

    lines = []

    clf = joblib.load("/home/manuel/catkin_ws/src/self_driving_car_thesis/scripts/classifiers/ang2ang_clf.pkl")

    for line in lines_h:
      for x1, y1, x2, y2 in line:
        lines.append(line.tolist())
        # ang_img = angle[y1-20:y2+20, x1-20:x2+20]
        # mag_img = mag[y1-20:y2+20, x1-20:x2+20]

        # if ang_img.shape[0] and ang_img.shape[1]:

        #   ang_hist = self.get_histogram(mag_img, ang_img, hist_type="ang2ang")
        #   pred = clf.predict(ang_hist)
        #   if pred == 1 or pred == 2:
        #     lines.append(line.tolist())

    rospy.set_param('/detected_lines', lines)

  def scaleTo(self, data, max_val):
    data = data.astype(np.float)

    scaler = MinMaxScaler()
    scaler.fit(data)
    data = scaler.transform(data)

    data = (data*max_val).astype(np.uint8)

    return data

  def get_histogram (self, mag, angle, hist_type):

    #Angle respect to magnitude
    hist = np.zeros((36,))

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



    #hist = hist.astype(np.float)/sum(hist)

    return hist.reshape(1,-1)


def main(args):
  rospy.init_node('edge_detector', anonymous=True)
  rospy.loginfo("Edge detector on")
  edge_det = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)