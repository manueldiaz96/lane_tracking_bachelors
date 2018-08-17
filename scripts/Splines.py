#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function

import roslib
import sys
import rospy

import cv2

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splev, splrep

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class spline_calculator: 

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/Lane_image/image",Image,self.callback)
    self.image_pub = rospy.Publisher("/Spline_publisher/image",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") #image bgr 8 canales
    except CvBridgeError as e:
      print(e)

    cv_image = self.Spline(cv_image)
    is_bgr = len(cv_image.shape) == 3

    try:
      if is_bgr:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      else:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
    except CvBridgeError as e:
      print(e)

  def Spline(self,img):
    imgcopy=img.copy()

    left_line=rospy.get_param('/left_points')
    print(left_line)
        #Crate a spline withput give the knots
# m=np.array( [[27, 356],
#  [43, 349],
#  [68, 342],
#  [96, 335],
#  [111, 328],
#  [128, 321],
#  [165, 314],
#  [186, 307],
#  [212, 300],
#  [247, 293],
#  [268, 286],
#  [302, 279],
#  [316, 272],
#  [342, 265],
#  [367, 258],
#  [410, 251],
#  [455, 244],
#  [495, 237],
#  [531, 230],
#  [569, 223],
#  [602, 216],
#  [629, 209],
#  [655, 202],
#  [698, 195],
#  [745, 188]]
# )
# maxm= np.amax(m)
# minm= np.amin(m)
# mm = len(m)
# print(mm)
# x=np.zeros(mm)
# y=np.zeros(mm)
# for i in range(mm):
#   x[i]= (m[i,0])
  
#   y[i]= (m[i,1])

# print((x))
# print((y))
# plt.plot(x,y,'o')
# plt.show()
# k= 2;
# s=3;
# task= -1;
# #find the knot points for the x,y, the degree of spline, bspline coefficients
# spl=splrep(x,y,k=k,s=s)
# #evaluae spline
# ynew = splev(np.linspace(minm,maxm),spl)
# print(ynew)
# xnew=np.linspace(minm,maxm)
# plt.plot(x,y,'o',xnew,ynew)
# plt.show()
    return imgcopy

def main(args):
  rospy.init_node('spline_node', anonymous=True)
  rospy.loginfo("Spline Calcultor on")
  edge_det = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
