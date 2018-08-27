#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function

import roslib
import sys
import rospy

import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scipy.interpolate import splev, splrep, splprep, Rbf, interp1d, UnivariateSpline

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/Haar_edges/image",Image,self.callback)
    self.image_pub = rospy.Publisher("/Lane_image/image",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    cv_image = self.sliding_windows(cv_image)
    is_bgr = len(cv_image.shape) == 3

    try:
      if is_bgr:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      else:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
    except CvBridgeError as e:
      print(e)

  def sliding_windows(self, img):
    # Assuming you have created a warped binary image called "img"
    # Take a histogram of the bottom half of the image
    h, w = img.shape
    histogram = np.sum(img[2*h/3:,:], axis=0)
    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((img, img, img))
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines

    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Choose the number of sliding windows
    nwindows = 80
    # Set height of windows
    window_height = np.int(img.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    #Keep the value at t-1
    # Set the width of the windows +/- margin
    margin = 50
    # Set minimum number of pixels found to recenter window / increase - decrease 
    minpix = 0
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    left_lane_centers = [] #this hold the door 
    right_lane_centers = []

    leftx_M_ch = []
    rightx_M_ch = []
    lefty_M_ch = []
    righty_M_ch = []

    positionsR=[]
    positionsL=[]

    # Step through the windows one by one
    for window in range(nwindows/2):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # Draw the windows on the visualization image
        #cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,140,0), 2)
        #cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,140,0), 2) 
        #Try to reduce the noisy data using logical parameters


        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_M_ch = (nonzerox[good_left_inds])
            lefty_M_ch = (nonzeroy[good_left_inds])
        if len(good_right_inds) > minpix:        
            rightx_M_ch = (nonzerox[good_right_inds])
            righty_M_ch = (nonzeroy[good_right_inds])

        #rightx_M_ch = np.reshape(rightx_M_ch,(len(rightx_M_ch),)) 
        #righty_M_ch = np.reshape(righty_M_ch,(len(righty_M_ch),))
        print(rightx_M_ch)

        #lefty_M_ch= np.reshape(lefty_M_ch,(len(lefty_M_ch),))
        #leftx_M_ch= np.reshape(leftx_M_ch,(len(leftx_M_ch),))

       

        #Fit a second order polynomial to each
        ml, bl = np.polyfit(leftx_M_ch, lefty_M_ch, 1)
        mr, br = np.polyfit(rightx_M_ch, righty_M_ch, 1)



        y_val = (win_y_high-win_y_low)/2
        y_val += win_y_low 

        x_new_left= ((y_val+win_y_high)-bl)/ml
        x_new_right= ((y_val+win_y_high)-br)/mr 

        rightx_current=[x_new_right,y_val]
        leftx_current=[x_new_left,y_val]
        centerR = np.array([rightx_current, y_val])
        centerL = np.array([leftx_current, y_val])

        
        # leftx_ind=[]
        # rightx_ind=[]
        # y_ind=[]
        # for i in range(len(centerR)):   
        #     if((centerR[0,0]-centerR[i,0])>0):
        #         rightx_ind.append(centerR[i])
        #     if((y_val[i+1]-y_val[i])>0):
        #         y_ind[i].append(y_val[i])
        # for  i in range(len(centerL)):
        #     if((centerL[i+1,0]-(centerL[i,0]))>0):
        #         leftx_ind.append((centerL[i,0]))


        #centerL=np.sort(centerL)
        #centerR=np.sort(centerR)
        left_lane_centers.append(centerL.tolist()) #Before was a np.array
        right_lane_centers.append(centerR.tolist())

    for i in range(len(rightx_M_ch)):
        positionsR=np.array([rightx_M_ch[i],righty_M_ch[i]])
    for i in range (len(lefty_M_ch)):
        positionsL=np.array([leftx_M_ch[i],lefty_M_ch[i]])

    # print("shape PsR",(positionsR.shape))
    # positionsR=np.reshape(positionsR,(len(positionsR),))
    # positionsL=np.reshape(positionsL,(len(positionsL),))
    # # Concatenate the arrays of indices
    # left_lane_inds = np.concatenate(left_lane_inds)
    # right_lane_inds = np.concatenate(right_lane_inds)
    # print(positionsL)

    rospy.set_param('/left_points', left_lane_centers)
    rospy.set_param('/right_points', right_lane_centers)
    # Generate x and y values for plotting


    # left_line=rospy.get_param('~/left_points')
    # left_line_rng= len(left_line);
    # left_line=np.reshape(left_line,(left_line_rng,2))   
    # left_line_max=np.amax(left_line);
    # left_line_min=np.amin(left_line);
    # x_left=np.zeros(left_line_rng)
    # y_left=np.zeros(left_line_rng)
    # for i in range(left_line_rng):
    #   x_left[i]= (left_line[i,0])
    #   y_left[i]= (left_line[i,1])
    #   if(x_left[i]==0):
    #     x_left[i]=x_left[i-1]
    #   if (y_left[i]==0):
    #     y_left[i]=y_left[i-1]


    # # Extract left and right line pixel positions

    # # At this point, you're done! But here is how you can visualize the result as well:
    # ploty = np.linspace(0, img.shape[0]-1, img.shape[0] )
    # left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    # right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [30, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 30]

    for center in left_lane_centers:
      cv2.circle(out_img,(center[0],center[1]), 5, (255,0,0), -1)

    for center in right_lane_centers:
      cv2.circle(out_img,(center[0],center[1]), 5, (0,0,255), -1)

    return out_img






def main(args):
  rospy.init_node('lane_point_extract', anonymous=True)
  rospy.loginfo("Point Extractor on")
  edge_det = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)