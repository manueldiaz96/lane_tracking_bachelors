#!/usr/bin/env python
# coding=utf-8

#Dataset curvas y duckietown gazebo
#Histograma de 3 bins y media del último bin

from __future__ import print_function

import roslib
import sys
import rospy

import cv2
import numpy as np
from tracker import Tracker

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self, roi_option):

        #Create ROS objects for image handling
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.image_pub = rospy.Publisher("/lane_model/lane",Image, queue_size = 2)

        #Create where to store the polynomial fitting for each of the lines
        self.left_fit = np.zeros((3,3), dtype=np.double)
        self.right_fit = np.zeros((3,3), dtype=np.double)
        self.pts_raw = [0,0,0,0]

        #Create the haar-like feature kernel to filter the image
        self.kernel = np.array([[0,0,1,1,1,0,0],[0,0,1,1,1,0,0],[0,0,1,1,1,0,0],[0,0,1,1,1,0,0]], dtype=np.float32)
        kernel_mult = 10
        self.kernel = np.kron(self.kernel,np.ones((kernel_mult, kernel_mult)))
        self.kernel /= (self.kernel.shape[0]*self.kernel.shape[1])/2.5

        #Based on the video, a different ROI is used
        if roi_option == 1:
            self.roi_points = [[600, 530],[800, 530],[1152, 700],[300, 700]]
        elif roi_option == 2:
            self.roi_points = [[645, 540],[925, 540],[1260, 700],[390, 700]]
        else:
            self.roi_points = [[605-60, 360+100],[745+50, 360+100],[1202, 720-50],[200, 720-50]]

        #Define how many search windows are going to be used
        self.n_windows = 15
        #And how many kalman filters are going to be used, which is n_windows per line
        self.kalman_filters_left = []
        self.kalman_filters_right = []
        self.distances_p2p = []

        for i in range(self.n_windows):
            #Create and initialize each KF
            self.kalman_filters_right.append(Tracker())
            self.kalman_filters_left.append(Tracker())
            self.distances_p2p.append(0)

        self.filtersNotInitialized = True
        self.lastleftx = 0
        self.lastrightx = 0
        self.Predicted_leftx = 0
        self.Predicted_rightx = 0

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

        if self.filtersNotInitialized:

            self.filtersNotInitialized = False;
            _, w, _ = cv_image.shape
            val_left = 2*np.int(w/10)
            val_right = 8*np.int(w/10)

            for i in range(self.n_windows):
                self.kalman_filters_left[i].x_state = np.array([[val_left, 0]]).T
                self.kalman_filters_left[i].predict_only()
                self.kalman_filters_right[i].x_state = np.array([[val_right, 0]]).T
                self.kalman_filters_right[i].predict_only()


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

        roi_points = self.roi_points 

        #for point in roi_points:
          #print(point)
          #cv2.circle(img, (point[0],point[1]), 5, (255,0,0), -1)

        # # https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/

        rect = np.array(roi_points, dtype="float32")
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

        warped = cv2.filter2D(warped, -1, self.kernel)

        
        thresh1 = np.zeros_like(warped, dtype=np.uint8)

        #Apply to each vertical half, a threshold based on its brightest spots

        thresh2 = np.vstack((thresh1,thresh1))

        thresh2[:,:(warped.shape[1]/2)] = self.Threshold(warped[:,:warped.shape[1]/2])

        thresh2[:,(warped.shape[1]/2):] = self.Threshold(warped[:,warped.shape[1]/2:])

        #pts, pts_raw, pts_center, _ = self.findLanes(thresh1)

        #final, curv_rad, dst_from_center, prueba = self.visualLane(img, pts, pts_raw, pts_center, M)
        
        final = self.findLanes(thresh1)

        final = self.paintLane(final, img, M)

        return thresh2

        #return final


    def Threshold(self, gray_img):

        gray_img = cv2.blur(cv2.equalizeHist(gray_img),(5,5))

        _ ,thresh1 = cv2.threshold(gray_img,245,255,cv2.THRESH_BINARY)

        return np.vstack((gray_img,thresh1))

        #return thresh1




    def findLanes(self, top_down):

        h, w = top_down.shape

        #The only visible portions of the image are (in columns):
            #The second and third tenths of the image (100-300 px)
            #The seventh and eight tenths of the image (700-900 px)

        #These can be seen as the initial states for the lines (200 and 800)

        #print (w)
        #print (np.int(w/10), 3*np.int(w/10), 7*np.int(w/10), 9*np.int(w/10))

        top_down[:,:np.int(w/10)] = 0
        top_down[:,3*np.int(w/10):7*np.int(w/10)] = 0

        #top_down[:,] = 0
        #top_down[:,8*np.int(w/10)] = 0 

        #top_down[:,9*np.int(w/10):] = 0

        n_windows = self.n_windows
        window_height = np.int(top_down.shape[0]/n_windows)

        leftx_current = 2*np.int(w/10)
        rightx_current = 8*np.int(w/10)

        nonzeroy = np.array(top_down.nonzero()[0])
        nonzerox = np.array(top_down.nonzero()[1])

        margin = 100
        minpix = 50

        top_down = np.dstack((top_down, top_down, top_down))
        top_down = np.zeros_like(top_down)

        for window in range(n_windows):
            win_y_low = top_down.shape[0] - (window+1)*window_height
            win_y_high = top_down.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]

            y = (win_y_high - win_y_low)/2
            y += win_y_low

            if len(good_left_inds) > minpix:
                try:
                    leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
                except ValueError as e:
                    leftx_current = self.lastleftx 
                else:
                    self.lastleftx = leftx_current

                x_st = [leftx_current]
                z = np.expand_dims(x_st, axis=0).T
                self.kalman_filters_left[window].kalman_filter(z)
                xx = self.kalman_filters_left[window].x_state;
                self.Predicted_leftx = xx[0,0]

            else:
                self.kalman_filters_left[window].predict_only;
                self.Predicted_leftx = self.kalman_filters_left[window].x_state[0,0]
                
            point = (self.Predicted_leftx, y)            
            cv2.circle(top_down, point, 5, (0,0,255), -1)


            if len(good_right_inds) > minpix:
                try:
                    rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                except ValueError as e:
                    rightx_current = self.lastrightx 
                else:
                    self.lastrightx = rightx_current

                x_st = [rightx_current]
                z = np.expand_dims(x_st, axis=0).T
                self.kalman_filters_right[window].kalman_filter(z)
                xx = self.kalman_filters_right[window].x_state;
                self.Predicted_rightx = xx[0,0]

            else:
                self.kalman_filters_right[window].predict_only;
                self.Predicted_rightx = self.kalman_filters_right[window].x_state[0,0]

            point = (self.Predicted_rightx, y)            
            cv2.circle(top_down, point, 5, (0,0,255), -1)


            if len(good_right_inds) > minpix and len(good_left_inds) > minpix:
                self.distances_p2p[window] = self.Predicted_rightx - self.Predicted_leftx
                
            

            #print("Window:", window, "leftx_current:", leftx_current-self.Predicted_leftx, "rightx_current:", rightx_current-self.Predicted_rightx)

        return top_down

    def paintLane(self, warp_zero, image, perspective_M):

        newwarp = cv2.warpPerspective(warp_zero, np.linalg.inv(perspective_M), (image.shape[1], image.shape[0]))
        result = cv2.addWeighted(image, 0.3, newwarp, 1.5, 0)

        return result



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
        #leftx, lefty, rightx, righty = pts_raw

        #warp_zero[lefty, leftx] = [255, 0, 0]
        #warp_zero[righty, rightx] = [255, 0, 0]

        # for indx in xrange(pts_center.shape[1]):
        #   x, y = (pts_center[0,indx], pts_center[1,indx])
        #   cv2.circle(warp_zero, (x,y), 5, (0,0,255), -1)

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(warp_zero, np.linalg.inv(perspective_M), (image.shape[1], image.shape[0]))
        # Combine the result with the original image
        result = cv2.addWeighted(image, 0.5, newwarp, 1, 0)

        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 30./720 # meters per pixel in y dimension
        xm_per_pix = 3.7/700 # meters per pixel in x dimension

        # Fit new polynomials to x,y in world space
        ymax = float(image.shape[0])
        y_eval = ymax
        leftx = pts_raw[0]
        lefty = pts_raw[1]
        rightx = pts_raw[2]
        righty = pts_raw[3]
        left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
        right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
        # Calculate the new radii of curvature
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        # Now our radius of curvature is in meters
        # print(left_curverad, 'm', right_curverad, 'm')

        # print distance from center and radius on the image
        lane_center = (self.evalPoly(left_fit_cr, ymax*ym_per_pix) + self.evalPoly(right_fit_cr, ymax*ym_per_pix))/2.0
        car_center = image.shape[1]*xm_per_pix/2.0
        str1 = "Distance from center: {:2.2f} m".format(car_center-lane_center)
        str2 = "Radius of Curvature: {:2.2f} km".format((left_curverad+right_curverad)/2000.)
        curv_rad = (left_curverad+right_curverad)/2000.
        dst_from_center = car_center-lane_center
        cv2.putText(result,str1,(430,630), cv2.FONT_HERSHEY_DUPLEX, 1,(0,0,255))  
        cv2.putText(result,str2,(430,660), cv2.FONT_HERSHEY_DUPLEX, 1,(0,0,255))

        return result, curv_rad, dst_from_center, warp_zero
        #return result


def main(args):

    rospy.init_node('lane_model', anonymous=True)
    roi_option = rospy.get_param('/vid')
    rospy.loginfo("Lane Model on")
      
    ic = image_converter(roi_option)
      
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
  

if __name__ == '__main__':
    main(sys.argv)