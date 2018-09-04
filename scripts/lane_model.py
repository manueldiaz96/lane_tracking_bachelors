#!/usr/bin/env python
# coding=utf-8

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

    cv_image= self.find_lane(cv_image)

    is_bgr = len(cv_image.shape) == 3

    if True:
      try:
        if is_bgr:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        else:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
      except CvBridgeError as e:
        print(e)



  def find_lane(self, img):

    h, w, _ = img.shape

    vert_margin = rospy.get_param('/vert_margin')
    roi_points = rospy.get_param('/roi_points')

    roi_points[0][1] = roi_points[0][1] + vert_margin + 10
    roi_points[1][1] = roi_points[1][1] + vert_margin + 10
    roi_points[2][1] = roi_points[2][1] + vert_margin + 10
    roi_points[3][1] = roi_points[3][1] + vert_margin + 10

    if np.mean(roi_points):

      # https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/

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

      for point in roi_points:
        cv2.circle(img, (point[0], point[1]), 10, (0,255,0), -1)

      res = cv2.resize(img,(warped.shape[1], h), interpolation = cv2.INTER_CUBIC)

      img = np.vstack((warped, res))
    
    return img

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