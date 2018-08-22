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
from scipy.interpolate import splev, splrep, splprep, Rbf, interp1d, UnivariateSpline

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
    
    left_line=rospy.get_param('~/left_points')
    left_line_rng= len(left_line);
    left_line=np.reshape(left_line,(left_line_rng,2))   
    left_line_max=np.amax(left_line);
    left_line_min=np.amin(left_line);
    x_left=np.zeros(left_line_rng)
    y_left=np.zeros(left_line_rng)
    for i in range(left_line_rng):
      x_left[i]= (left_line[i,0])
      y_left[i]= (left_line[i,1])
      if(x_left[i]==0):
        x_left[i]=x_left[i-1]
      if (y_left[i]==0):
        y_left[i]=y_left[i-1]
    #x_left=np.sort(x_left)
    #y_left=np.sort(y_left)
    #k=3;
    #s=0.8;
    #find the knot points for the x,y, the degree of spline, bspline coefficients
    #tckl,u=splprep([x_left,y_left],k=k)
    #Rbf spline. Works bad - could be good    
    spl_l=Rbf(x_left,y_left, smooth=0.1) 
    #1-D Interpolation 
    #pld1_l=interp1d(x_left,y_left,kind='cubic') donts works. 
    #unv_l=UnivariateSpline(x_left,y_left,k=3,s=0.5)

    #np.sort(unv_l)


    xnew_left=np.linspace(np.amin(x_left),np.amax(x_left))
    #xnew_left=np.sort(xnew_left)
    #ynew_left = splev(xnew_left,tckl)
    rbfs=spl_l(xnew_left)
    rbfs=rbfs.tolist()
    rospy.set_param('rbfs',rbfs)
    np.floor(rbfs)
    #unv_l=unv_l(xnew_left)
    #np.floor(unv_l)
    #np.floor(pld1_l)
    rbfs=np.asarray(rbfs,dtype=int)
    #pld1_l=np.asarray(pld1_l,dtype=int)
    #ynew_left=np.asarray(ynew_left, dtype=int)
    #unv_l=np.asarray(unv_l,dtype=int)
    xnew_left=np.asarray(xnew_left, dtype=int)

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
      if(x_rigth[i] == 0):
        x_rigth[i]=x_rigth[i-1]
      if(y_right[i]==0):
        y_right[i]=y_right[i-1] #Shoul it be a medium value, 
    k= 2;
    s= 3;

    
    #x_rigth=np.sort(x_rigth)
    #y_right=np.sort(y_right)
    #pld1_r=interp1d(x_rigth,y_right, kind='cubic')
    #find the knot points for the x,y, the degree of spline, bspline coefficients
    #tckr,u=splprep([x_rigth,y_right],k=k,s=s)
    #unv_r=UnivariateSpline(x_rigth,y_right,k=3,s=0.5)
    #np.sort(unv_r)
    xnew_right=np.linspace(np.amin(x_rigth),np.amax(x_rigth))
    #xnew_right=np.sort(xnew_right)
    #unv_r=unv_r(xnew_right)
    #np.floor(unv_r)

    #np.floor(pld1_r)
    #pld1_r=np.asarray(pld1_r,dtype=int)
    #unv_r=np.asarray(unv_r,dtype=int)

    rbfs_r=Rbf(x_rigth,y_right, smooth=0.1)
    rbfs_r=rbfs_r(xnew_right)
    rbfs_r=rbfs_r.tolist()
    rospy.set_param('rbfs_r',rbfs_r)
    np.floor(rbfs_r)
    rbfs_r=np.asarray(rbfs_r,dtype=int)
    #ynew_right = splev(xnew_right,tckr)
    #ynew_right=np.asarray(ynew_right, dtype=int)
    #print(*ynew_right)
    #xnew_right=xnew_right.tolist()
    #rospy.set_param('/xnew_right',xnew_right)
    #ynew_right=ynew_right.tolist()
    #rospy.set_param('/ynew_right',ynew_right)
    xnew_right=np.asarray(xnew_right, dtype=int)

    #ynew_right.astype(int)
    #ynew_left.astype(int)
    #ynew_right= ynew_right.tolist()
    #rospy.set_param('/ynew_right',ynew_right)

    for i in range (0,right_line_rng):
      cv2.line(imgcopy,(xnew_right[i],rbfs_r[i]),(xnew_right[i+1],rbfs_r[i+1]),(0,0,255),2)
    for i in range (0,left_line_rng):
      cv2.line(imgcopy,(xnew_left[i],rbfs[i]),(xnew_left[1+i],rbfs[1+i]),(255,0,0),2)

    # for i in range (0,left_line_rng):
    #    cv2.line(imgcopy,(((xnew_left[i]),(ynew_left[i]))),((int(xnew_left[i+1]),int(ynew_left[i+1]))),(255,0,0), 2)
    

    # for i in range(right_line_rng):
    #    cv2.line(imgcopy,((int(xnew_right[i]),int(ynew_right[i]))),((int(xnew_right[i+1]),int(ynew_right[i+1]))),(0,0,255), 2)
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
