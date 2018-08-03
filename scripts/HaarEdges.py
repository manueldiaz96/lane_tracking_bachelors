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

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.image_pub = rospy.Publisher("/Haar_edges/Kernel1",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image = self.HaarEdges(cv_image)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
    except CvBridgeError as e:
      print(e)

  def HaarEdges(self, img):
    h, w, _ = img.shape

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask =  (gray.copy())
    mask2 = mask

    ddepth = -1
    #kernel = self.getKernel("diag_Left")
    kernel =np.array([[0,0,1],[0,1,0],[1,0,0]],dtype=np.float32)

    #kernel = np.kron(kernel, np.ones((5,5)))
    kernel /= kernel.shape[0]*kernel.shape[1]

    mask[:, 0:w/2] = cv2.filter2D(mask[: ,0:w/2], ddepth, kernel)

    #kernel = self.getKernel("diag_Right")
    kernel =np.array([[1,0,0],[0,1,0],[0,0,1]],dtype=np.float32)

    #kernel = np.kron(kernel, np.ones((5,5)))
    kernel /= kernel.shape[0]*kernel.shape[1]

    mask[:, (w/2) : w-1] = cv2.filter2D(mask[:,(w/2) : w-1], ddepth, kernel)

    _,th2 = cv2.threshold(mask,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    th2 = cv2.dilate(th2, cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)), iterations = 2) 

    th3 = 255-cv2.adaptiveThreshold(mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,21,2)
    #21*(h/24):h-1,(w/2)-10:(w/2)+10

    th3 = cv2.bitwise_and(th2, th3)

    img = th3
    #img[:, w/2:(w/2)+1] = 255
    #img = np.vstack((np.hstack((th2, th3))[5*(h/12) : 22*(h/24), :],np.hstack((mask, gray))[5*(h/12) : 22*(h/24), :]))

    #print(th2[22*(h/24):h-1,(w/2)-10:(w/2)+10])

    #return img
    return img[5*(h/12) : 22*(h/24), :]


  def getKernel(self, kernelID):
    Kernel_dict ={
    "diag_Left": np.array([[0,0,0,1,1],
                           [0,0,1,1,1],
                           [0,1,1,1,0],
                           [1,1,1,0,0],
                           [1,1,0,0,0]], dtype=np.float32),

    "diag_Right": np.array([[1,1,0,0,0],
                            [1,1,1,0,0],
                            [0,1,1,1,0],
                            [0,0,1,1,1],
                            [0,0,0,1,1]], dtype=np.float32),

    "Up": np.array([[1,1,1,1,1],
                    [1,1,1,1,1],
                    [1,1,1,1,1],
                    [0,0,0,0,0],
                    [0,0,0,0,0]], dtype=np.float32),

    "Down": np.array([[0,0,0,0,0],
                      [0,0,0,0,0],
                      [1,1,1,1,1],
                      [1,1,1,1,1],
                      [1,1,1,1,1]], dtype=np.float32),

    "Left": np.array([[1,1,1,0,0],
                      [1,1,1,0,0],
                      [1,1,1,0,0],
                      [1,1,1,0,0],
                      [1,1,1,0,0]], dtype=np.float32),

    "Right": np.array([[0,0,1,1,1],
                       [0,0,1,1,1],
                       [0,0,1,1,1],
                       [0,0,1,1,1],
                       [0,0,1,1,1]], dtype=np.float32),

    "sq_Left": np.array([[0,0,0,1,1],
                         [0,0,0,1,1],
                         [0,0,1,0,0],
                         [1,1,0,0,0],
                         [1,1,0,0,0]], dtype=np.float32),

    "sq_Right": np.array([[1,1,0,0,0],
                          [1,1,0,0,0],
                          [0,0,1,0,0],
                          [0,0,0,1,1],
                          [0,0,0,1,1]], dtype=np.float32)

    }

    return Kernel_dict.get(kernelID, np.zeros((5,5), dtype=np.float32))

def main(args):
  rospy.init_node('HaarEdges_node', anonymous=True)
  rospy.loginfo("HaarEdges_node")
  edge_det = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)