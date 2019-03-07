#!/usr/bin/env python
# coding=utf-8


from __future__ import print_function

import roslib
import sys
import rospy

import cv2
import numpy as np
from KF_tracker import Tracker

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self, roi_option):

        #Create ROS objects for image handling
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)
        self.image_pub = rospy.Publisher("/lane_model/lane",Image, queue_size = 2)
        self.steer_angle = rospy.Publisher("/steer_angle_img", Float64, queue_size=1)

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
            self.roi_points = [[625, 540],[935, 540],[1300, 700],[340, 700]]
        else:
            self.roi_points = [[545, 460],[770, 460],[1202, 670],[200, 670]]


        #Define how many search windows are going to be used
        self.n_windows = 10
        #And how many kalman filters are going to be used, which is n_windows per line
        self.kalman_filters_left = []
        self.kalman_filters_right = []
        self.distances_p2p = []
        self.middlePoints = []

        self.left_points_for_std = []
        self.right_points_for_std = []


        for i in range(self.n_windows):
            #Create and initialize each KF
            self.kalman_filters_right.append(Tracker())
            self.kalman_filters_left.append(Tracker())

            self.distances_p2p.append(0)
            self.middlePoints.append(0)

            self.left_points_for_std.append([])
            self.right_points_for_std.append([])

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

        _, w, _ = cv_image.shape

        if self.filtersNotInitialized:

            self.filtersNotInitialized = False

            val_left = 2*np.int(w/10)
            val_right = 8*np.int(w/10)

            for i in range(self.n_windows):
                self.kalman_filters_left[i].x_state = np.array([[val_left, 0]]).T
                self.kalman_filters_left[i].predict_only()
                self.kalman_filters_right[i].x_state = np.array([[val_right, 0]]).T
                self.kalman_filters_right[i].predict_only()


        cv_image = self.lane_detect(cv_image)

        #error = self.calculateError(w/2)

        #cv_image = self.sendControlCommand(error, cv_image)

        is_bgr = len(cv_image.shape) == 3

        if True:
          try:
            if is_bgr:
              self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            else:
              self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
          except CvBridgeError as e:
            print(e)

    def lane_detect(self, img):

        h, w, _ = img.shape

        roi_points = self.roi_points 

        # # https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/

        warped, M, maxPts = self.warp_lane(roi_points, img)

        warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

        warped = cv2.filter2D(warped, -1, self.kernel)

        thresh1 = np.zeros_like(warped, dtype=np.uint8)

        #Apply to each vertical half, a threshold based on its brightest spots

        thresh1[:,:(warped.shape[1]/2)] = self.Threshold(warped[:,:warped.shape[1]/2])

        thresh1[:,(warped.shape[1]/2):] = self.Threshold(warped[:,warped.shape[1]/2:])
        
        final = self.findLanes(thresh1)

        final = self.paintLane(final, img, M)

        middlePoints = np.dstack((self.middlePoints[:,0],self.middlePoints[:,1]))

        middlePoints = cv2.perspectiveTransform(middlePoints, np.linalg.inv(M)).astype(np.int)


        for point in range(middlePoints.shape[1]):
            self.middlePoints[point] = middlePoints[0,point,:]
            x, y = self.middlePoints[point]
            #print(x,y)
            #cv2.circle(final, (x, y), 5, (0,255,255), -1)

        #final[:,np.int(w/2)]=(255,125,0)

        error = self.calculateError(w/2)

        final = self.sendControlCommand(error, img, M, maxPts)

        return final

        #final = self.paintRoi(final, roi_points)

        #return img

    def warp_lane(self, roi_points, img):

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

        return warped, M, (maxWidth, maxHeight)

    def Threshold(self, gray_img):

        gray_img = cv2.blur(cv2.equalizeHist(gray_img),(5,5))

        gamma = 20
        gray_img2 = (np.power(gray_img.astype(np.float)/255, gamma) * 255).astype(np.uint8)

        mean = np.uint8(15*np.mean(gray_img2))
        #print(18*mean)

        _ ,thresh1 = cv2.threshold(gray_img,240,250,cv2.THRESH_BINARY)

        return thresh1
        #return gray_img

    def findLanes(self, top_down):

        h, w = top_down.shape

        #The only visible portions of the image are (in columns):
            #The second and third tenths of the image (100-300 px)
            #The seventh and eight tenths of the image (700-900 px)

        #These can be seen as the initial states for the lines (200 and 800)

        #print (w)
        #print (np.int(w/10), 3*np.int(w/10), 7*np.int(w/10), 9*np.int(w/10))

        top_down[:,:np.int(w/20)] = 0
        top_down[:,3*np.int(w/10):7*np.int(w/10)] = 0
        top_down[:,19*np.int(w/20):] = 0

        #top_down[:,] = 0
        #top_down[:,8*np.int(w/10)] = 0 

        

        n_windows = self.n_windows
        window_height = np.int(top_down.shape[0]/n_windows)

        leftx_current = 2*np.int(w/10)
        rightx_current = 8*np.int(w/10)

        nonzeroy = np.array(top_down.nonzero()[0])
        nonzerox = np.array(top_down.nonzero()[1])

        margin = 100
        minpix = 100

        top_down = np.dstack((top_down, top_down, top_down))
        top_down = np.zeros_like(top_down)

        #print(np.array(self.right_std, dtype=np.int))
        #print(self.left_points_for_std)


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
                    leftx_current = (self.lastleftx+leftx_current)/2
                else:
                    self.lastleftx = leftx_current

                #######################################################################################

                store_size = 5
                
                if len(self.left_points_for_std[window]) < store_size:
                    self.left_points_for_std[window].append(leftx_current)

                if len(self.left_points_for_std[window]) == store_size:
                    self.left_points_for_std[window] = self.getMeanStd(self.left_points_for_std[window], leftx_current)


                #######################################################################################

                #leftx_current = self.left_points_for_std[window][-1]

                x_st = [leftx_current]
                z = np.expand_dims(x_st, axis=0).T
                self.kalman_filters_left[window].kalman_filter(z)
                self.Predicted_leftx = self.kalman_filters_left[window].x_state[0,0]
                color = (0,0,255)
                

            else:
                # if False: #len(good_left_inds) > minpix:
                #     self.Predicted_leftx = self.Predicted_rightx - self.distances_p2p[window]
                # else:
                self.kalman_filters_left[window].predict_only;
                self.Predicted_leftx = self.kalman_filters_left[window].x_state[0,0]
                color = (0,255,0)
                
            point = (self.Predicted_leftx, y)            
            cv2.circle(top_down, point, 5, color, -1)


            if len(good_right_inds) > minpix:
                try:
                    rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                except ValueError as e:
                    rightx_current = (self.lastrightx+rightx_current)/2 
                else:
                    self.lastrightx = rightx_current

                #######################################################################################
                
                store_size = 5


                if len(self.right_points_for_std[window]) < store_size:
                    self.right_points_for_std[window].append(rightx_current)
                    #print(len(self.right_points_for_std[window]))

                if len(self.right_points_for_std[window]) == store_size:
                    self.right_points_for_std[window] = self.getMeanStd(self.right_points_for_std[window], rightx_current)

                #######################################################################################

                #rightx_current = self.right_points_for_std[window][-1]

                x_st = [rightx_current]
                z = np.expand_dims(x_st, axis=0).T
                self.kalman_filters_right[window].kalman_filter(z)
                self.Predicted_rightx = self.kalman_filters_right[window].x_state[0,0]
                color = (0,0,255)
                

            else:
                # if False: #len(good_left_inds) > minpix:
                #     self.Predicted_rightx = self.Predicted_leftx + self.distances_p2p[window]
                # else:
                self.kalman_filters_right[window].predict_only;
                self.Predicted_rightx = self.kalman_filters_right[window].x_state[0,0]
                color = (255,0,0)

            point = (self.Predicted_rightx, y)            
            cv2.circle(top_down, point, 5, color, -1)



            point = (self.Predicted_leftx+(self.Predicted_rightx-self.Predicted_leftx)/2, y)
            color = (0,255,255)
            #cv2.circle(top_down, point, 5, color, -1)

            #top_down[:,w/2]=(0,255,255)

            self.middlePoints[window] = [point[0],point[1]]


            if len(good_right_inds) > minpix and len(good_left_inds) > minpix:
                self.distances_p2p[window] = self.Predicted_rightx - self.Predicted_leftx
            

            #print("Window:", window, "leftx_current:", leftx_current-self.Predicted_leftx, "rightx_current:", rightx_current-self.Predicted_rightx)

        #cv2.circle(top_down, (np.int(w/2),np.int(h/2)), 15, (0,128,128), -1)  
        self.middlePoints = np.float32(self.middlePoints)

        return top_down

    def paintLane(self, warp_zero, image, perspective_M):

        newwarp = cv2.warpPerspective(warp_zero, np.linalg.inv(perspective_M), (image.shape[1], image.shape[0]))
        result = cv2.addWeighted(image, 0.2, newwarp, 0.8, 0)

        return result

    def paintRoi(self, img, roi_points):

        for point in roi_points:
          #print(point)
          cv2.circle(img, (point[0],point[1]), 5, (255,0,0), -1)

        cv2.line(img, (roi_points[0][0],roi_points[0][1]), (roi_points[1][0],roi_points[1][1]), (255,0,0), 3)
        cv2.line(img, (roi_points[1][0],roi_points[1][1]), (roi_points[2][0],roi_points[2][1]), (255,0,0), 3)
        cv2.line(img, (roi_points[2][0],roi_points[2][1]), (roi_points[3][0],roi_points[3][1]), (255,0,0), 3)
        cv2.line(img, (roi_points[3][0],roi_points[3][1]), (roi_points[0][0],roi_points[0][1]), (255,0,0), 3)

        return img

    def calculateError(self, w_2):

        error = 0

        for point in range(self.middlePoints.shape[0]):
            error += (1/(point+1))*(self.middlePoints[point][0]-w_2)

        error = np.int(error)
        #print(error)

        return error

    def getMeanStd(self, points, lastPoint):

        mean = np.mean(points)
        std = np.std(points)

        append = (lastPoint > (mean-(10*std))) and  (lastPoint < (mean+(10*std)))

        if append:
            points.remove(points[0])
            points.append(lastPoint)
            #points = np.delete(points, 0)
            #points = np.append(points, lastPoint)

        return points

    def sendControlCommand(self, error, img, M, maxPts):

        h, w, _ = img.shape

        pt1 = (w/2, h-1)

        pt2 = ((w/2)+error, 2*h/3)

        num = pt2[0]-pt1[0]

        den = np.sqrt((pt2[1]-pt1[1])**2 + (pt2[0]-pt1[0])**2)

        theta = np.arcsin(num/den)

        self.steer_angle.publish(theta)

        #pt1_w = np.dstack((pt1[0], pt1[1]))

        #pt2_w = np.dstack((pt2[0], pt2[1]))

        #pt1_w = cv2.perspectiveTransform(pt1_w, M).astype(np.int)

        #pt2_w = cv2.perspectiveTransform(pt2_w, M).astype(np.int)

        #for point in range(pt1_w.shape[1]):
            #pt1 = pt1_w[0,point,:]
            #pt2 = pt2_w[0,point,:]

        #arrow =  np.zeros_like(cv2.warpPerspective(img, M, maxPts))

        arrow = cv2.arrowedLine( img, pt1, pt2, (128,64,134),  thickness=3, line_type=cv2.FILLED)

        #newwarp = cv2.warpPerspective( arrow, np.linalg.inv(M), (w, h))

        #result = cv2.addWeighted(img, 0.3, newwarp, 1.5, 0)

        #result = cv2.arrowedLine( result, pt1, pt2, (128,64,134),  thickness=3, line_type=cv2.FILLED)

        return arrow




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
