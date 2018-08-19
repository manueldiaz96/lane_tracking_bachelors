#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function

import roslib
import sys
import os
import rospy

import cv2

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splev, splrep, splprep

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter: 

  def __init__(self):
    self.image_pub = rospy.Publisher("/Spline_publisher/image",Image,queue_size = 2)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/Lane_image/image",Image,self.callback)
    

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") #image bgr 8 canales
    except CvBridgeError as e:
      print(e)

    cv_image = self.Splines(cv_image)
    is_bgr = len(cv_image.shape) == 3

    try:
      if is_bgr:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      else:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
    except CvBridgeError as e:
      print(e)

  def Splines(self,img):
    imgcopy=img.copy()
    out_img = np.dstack((img, img, img))

    left_line=rospy.get_param('~/left_points')
    left_line_rng= len(left_line);
    left_line=np.reshape(left_line,(2,left_line_rng))   
    left_line_max=np.amax(left_line);
    left_line_min=np.amin(left_line);
    x_left=np.zeros(left_line_rng)
    y_left=np.zeros(left_line_rng)
    for i in range(left_line_rng):
      x_left[i]= (left_line[0,i])
      y_left[i]= (left_line[1,i])
    k=2;
    s=3;
    #find the knot points for the x,y, the degree of spline, bspline coefficients
    tckl,u=splprep([x_left,y_left],k=k)
    xnew_left=np.linspace(left_line_min,left_line_max)
    ynew_left = splev(xnew_left,tckl)
    ynew_left=np.asarray(ynew_left)
    for i in range(left_line_rng):
      int(ynew_left(i))

    ##Problemas no puedo obtener el valor del ynew para saber que formato esta y modificar asi xnew.
    ##No puedo generar la linea entre los ynew y xnew que conforma la spline. 
    ##no puedo imprimir con print, os.write para ver variables, solo por set_param. Help
    #ynew_left=ynew_left.tolist()
    #rospy.set_param('/ynew_left',ynew_left)

    right_line=rospy.get_param('~/right_points')
    right_line_rng= len(right_line);
    right_line=np.reshape(right_line,(right_line_rng,2))
    right_line_max=np.amax(right_line);
    right_line_min=np.amin(right_line);
    x_rigth=np.zeros(right_line_rng)
    y_right=np.zeros(right_line_rng)
    for i in range(right_line_rng):
      x_rigth[i]= (right_line[i,0])
      y_right[i]= (right_line[i,1])
    k= 2;
    s= 3;
    
    #find the knot points for the x,y, the degree of spline, bspline coefficients
    tckr,u=splprep([x_rigth,y_right],k=k,s=s)
    xnew_right=np.linspace(right_line_min,right_line_max)
    ynew_right = splev(xnew_right,tckr)
    ynew_right=np.asarray(ynew_right)
    xnew_right=xnew_right.tolist()
    rospy.set_param('/xnew_right',xnew_right)
    #ynew_right= ynew_right.tolist()
    #rospy.set_param('/ynew_right',ynew_right)

    for i in range (0,left_line_rng):
       cv2.line(imgcopy,int(xnew_left[i]),int(ynew_left[i]),int(xnew_left[i+1]),int(ynew_left[i+1]),(255,0,0), 2)
       if i+1 ==left_line_max:
         cv2.line(imgcopy,(int(xnew_left[i]),int(ynew_left[i])),(int(xnew_left[i+1]),int(ynew_left[i+1])),(255,0,0), 2)
         break

    for i in range(right_line_rng):
       cv2.line(imgcopy,(int(xnew_right[i],ynew_right[i])),(int(xnew_right[i+1],ynew_right[i+1])),(0,0,255), 2)
    return imgcopy


# xnew=np.linspace(minm,maxm)
# plt.plot(x,y,'o',xnew,ynew)
# plt.show()

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
