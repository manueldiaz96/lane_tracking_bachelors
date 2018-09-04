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
    out_img = np.dstack((img, img, img))*255
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    num_margin = 100
    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.int(w/3)+50
    rightx_base = np.int(3*w/4)

    # Choose the number of sliding windows
    nwindows = 20
    # Set height of windows
    window_height = np.int(img.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Set the width of the windows +/- margin
    
    # Set minimum number of pixels found to recenter window
    minpix = 10
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    left_lane_centers = []
    right_lane_centers = []

    x1, x2, ml, bl = rospy.get_param('/xleft_lane')

    img = np.dstack((img, img, img))

    # Step through the windows one by one
    for window in range(nwindows/2):
        # Identify window boundaries in x and y (and right and left)
        if window > 0:
            margin = np.int(num_margin*0.92)
            num_margin = margin
        else:
            margin = np.int(num_margin*1.5)

        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # Draw the windows on the visualization image
        #cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,140,0), 2)
        #cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,140,0), 2) 

        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        nonzero_left_x = nonzerox[good_left_inds]
        nonzero_left_y = nonzeroy[good_left_inds]

        nonzero_right_x = nonzerox[good_right_inds]
        nonzero_right_y = nonzeroy[good_right_inds]

        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
        
            x_size = nonzero_left_x.shape[0]

            #x1 = left_lane_centers 

            x1 = np.float(np.mean(nonzero_left_x[:np.int(x_size/2)]))
            x2 = np.float(np.mean(nonzero_left_x[np.int(x_size/2):]))

            if (x1!=x2):
                ml = ((x2 - x1)/(win_y_high-win_y_low))
                bl = x1 - ml*win_y_low
                
                rospy.set_param('/xleft_lane', [x1, x2, ml, bl])

                next_y = (img.shape[0] - (window+1)*window_height)-(img.shape[0] - (window+2)*window_height)
                next_y = (next_y/2) + (img.shape[0] - (window+2)*window_height)
                leftx_current = np.int(ml*next_y + bl)

                cv2.line(img, (np.int(x1), win_y_high), (leftx_current, next_y), [0, 200, 0], 3)
            #ml, bl = np.polyfit(nonzero_left_y, nonzero_left_x, 1)
            #leftx_current = (ml*(window+2)*window_height )+ bl
            #leftx_current = np.int(leftx_current)

        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))


        y_val = (win_y_high-win_y_low)/2
        y_val += win_y_low 
        centerR = np.array([rightx_current, y_val])
        centerL = np.array([leftx_current, y_val])

        left_lane_centers.append(centerL.tolist())
        right_lane_centers.append(centerR.tolist())

            

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    rospy.set_param('/left_points', left_lane_centers)
    rospy.set_param('/right_points', right_lane_centers)

    

    # # Extract left and right line pixel positions
    # leftx = nonzerox[left_lane_inds]
    # lefty = nonzeroy[left_lane_inds] 
    # rightx = nonzerox[right_lane_inds]
    # righty = nonzeroy[right_lane_inds] 

    # # Fit a second order polynomial to each
    # left_fit = np.polyfit(lefty, leftx, 2)
    # right_fit = np.polyfit(righty, rightx, 2)
    # # At this point, you're done! But here is how you can visualize the result as well:
    # # Generate x and y values for plotting
    # ploty = np.linspace(0, img.shape[0]-1, img.shape[0] )
    # left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    # right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    

    img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [130, 0, 0]
    img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 130]

    for center in left_lane_centers:
      cv2.circle(img,(center[0],center[1]), 5, (255,0,0), -1)

    for center in right_lane_centers:
      cv2.circle(img,(center[0],center[1]), 5, (0,0,255), -1)

    out_img[:,np.int(w/2)] = (255, 255, 255)
    out_img[:,np.int(w/4)] = (127,127,127)
    out_img[:,np.int(3*w/4)] = (127,127,127)

    
    return img


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