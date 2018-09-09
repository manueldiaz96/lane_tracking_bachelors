#!/usr/bin/env python
# coding=utf-8

#Dataset curvas y duckietown gazebo

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
    self.image_pub = rospy.Publisher("/lane_model/points",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image= self.warp_lane(cv_image)

    is_bgr = len(cv_image.shape) == 3

    if True:
      try:
        if is_bgr:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        else:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
      except CvBridgeError as e:
        print(e)



  def warp_lane(self, img):

    h, w, _ = img.shape

    h_delta = 5*(h/12)

    #vert_margin = rospy.get_param('/vert_margin')
    #roi_points = rospy.get_param('/roi_points')

    #roi_points = [[715, 210+h_delta],[825, 210+h_delta],[1252, 400+h_delta],[371, 400+h_delta]]

    roi_points2 = [[605-20, np.int(h/2)+100],[745+20, np.int(h/2)+100],[1202, h-50],[200, h-50]]

    #print(np.mean(roi_points))

      # https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/

    # for point in roi_points2:
    #   #print(point)
    #   cv2.circle(img, (point[0],point[1]), 5, (255,0,0), -1)

    rect = np.array(roi_points2, dtype="float32")
    (tl, tr, br, bl) = rect

    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    dst = np.array([[0, 0], [maxWidth - 1, 0], [maxWidth - 1, maxHeight - 1], [0, maxHeight - 1]], dtype = "float32")

    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(img, M, (maxWidth, maxHeight))

    warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

    thr = np.int(np.mean(warped)) + 30

    _ ,thresh1 = cv2.threshold(warped,thr,255,cv2.THRESH_BINARY)

    #warped = cv2.Sobel(thresh1,cv2.CV_64F,1,0,ksize=5)

    thr = np.int(np.mean(warped)) + 20

    _ ,thresh1 = cv2.threshold(warped,thr,255,cv2.THRESH_BINARY)

    pts, pts_raw, pts_center, thresh1 = self.findLanes(thresh1)

    #final, curv_rad, dst_from_center, prueba = self.visualLane(img, pts, pts_raw, pts_center, M)

    final = self.visualLane(img, pts, pts_raw, pts_center, M)
    
    return final


  def findLanes(self, top_down):

    """
    extract lanes from top_down view of the road
    """
    binary_warped = np.zeros((top_down.shape[0], top_down.shape[1]))
    binary_warped[(top_down[:,:]>0)] = 1

    # Assuming you have created a warped binary image called "binary_warped"
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]/2:,:], axis=0)
    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    out_img = np.uint8(out_img)
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int(binary_warped.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()

    use_past_polyfit = (nonzero[0].shape[0]>0.3*top_down.shape[0]*top_down.shape[1])

    if not use_past_polyfit:
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 50
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
            (0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
            (0,255,0), 2) 
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 
        pts_raw = [leftx, lefty, rightx, righty]

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        rospy.set_param('/polyfit_k3', rospy.get_param('/polyfit_k2'))
        rospy.set_param('/polyfit_k2', rospy.get_param('/polyfit_k1'))
        rospy.set_param('/polyfit_k1', [left_fit.tolist(), right_fit.tolist()])
        #rospy.set_param('/pts_raw', [leftx.tolist(), lefty.tolist(), rightx.tolist(), righty.tolist()])


    else:
        left_fit_k1, right_fit_k1 = np.array((rospy.get_param('/polyfit_k1')))
        left_fit_k2, right_fit_k2 = np.array((rospy.get_param('/polyfit_k2')))
        left_fit_k3, right_fit_k3 = np.array((rospy.get_param('/polyfit_k3')))

        left_fit = np.mean((left_fit_k1,left_fit_k2,left_fit_k3), axis=0)
        right_fit = np.mean((right_fit_k1,right_fit_k2,right_fit_k3), axis=0)

        #pts_raw = [leftx, lefty, rightx, righty]
        pts_raw = 0


    # Generate x and y values for plotting
    ploty = np.linspace(0, top_down.shape[0]-1, top_down.shape[0] )
    left_fitx = self.evalPoly(left_fit, ploty)
    right_fitx = self.evalPoly(right_fit, ploty)
    #print("Left x fit:", left_fitx)
    #print("Right x fit:", left_fitx)
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))], dtype=np.int32)
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))], dtype=np.int32)
    pts_center = np.vstack((((pts_right[0,:,0] - pts_left[0,:,0])/2) + pts_left[0,:,0],pts_right[0,:,1]))

    #pts_center = 
    pts = np.hstack((pts_left, pts_right))

    return pts, pts_raw, pts_center, out_img
    #return out_img

  def evalPoly(self, fit_param, Y):
    """
    Evaluate X, based on Y of the polynomial
    """
    return fit_param[0]*Y**2 + fit_param[1]*Y + fit_param[2]

  def visualLane(self, image, pts, pts_raw, pts_center, perspective_M):
    """
    Visualize the detected lane, radius, and car center shift
    """
    # plot on original image
    # Create an image to draw the lines on
    warp_zero = np.zeros_like(image).astype(np.uint8)

    # Draw the lane onto the warped blank image
    cv2.fillPoly(warp_zero, pts, (0,255, 0))

    for indx in xrange(pts_center.shape[1]):
      x, y = (pts_center[0,indx], pts_center[1,indx])
      cv2.circle(warp_zero, (x,y), 5, (0,0,255), -1)

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(warp_zero, np.linalg.inv(perspective_M), (image.shape[1], image.shape[0]))
    # Combine the result with the original image
    result = cv2.addWeighted(image, 1, newwarp, 0.5, 0)

    # Define conversions in x and y from pixels space to meters
    # ym_per_pix = 30./720 # meters per pixel in y dimension
    # xm_per_pix = 3.7/700 # meters per pixel in x dimension

    # # Fit new polynomials to x,y in world space
    # ymax = float(image.shape[0])
    # y_eval = ymax
    # leftx = pts_raw[0]
    # lefty = pts_raw[1]
    # rightx = pts_raw[2]
    # righty = pts_raw[3]
    # left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
    # right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
    # # Calculate the new radii of curvature
    # left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    # right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # # Now our radius of curvature is in meters
    # # print(left_curverad, 'm', right_curverad, 'm')

    # # print distance from center and radius on the image
    # lane_center = (self.evalPoly(left_fit_cr, ymax*ym_per_pix) + self.evalPoly(right_fit_cr, ymax*ym_per_pix))/2.0
    # car_center = image.shape[1]*xm_per_pix/2.0
    # str1 = "Distance from center: {:2.2f} m".format(car_center-lane_center)
    # str2 = "Radius of Curvature: {:2.2f} km".format((left_curverad+right_curverad)/2000.)
    # curv_rad = (left_curverad+right_curverad)/2000.
    # dst_from_center = car_center-lane_center
    # cv2.putText(result,str1,(430,630), cv2.FONT_HERSHEY_DUPLEX, 1,(0,0,255))  
    # cv2.putText(result,str2,(430,660), cv2.FONT_HERSHEY_DUPLEX, 1,(0,0,255))    
    # return result, curv_rad, dst_from_center, warp_zero
    return result


def main(args):
  rospy.init_node('lane_model', anonymous=True)
  rospy.loginfo("Lane Model on")
  ic = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  

if __name__ == '__main__':
    main(sys.argv)