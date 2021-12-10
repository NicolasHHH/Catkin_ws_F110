#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import pyplot as plt

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback) #/rgb
        

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # convert ros Image into cv2 format 
        except CvBridgeError as e:
            print(e)
            
        
        img = cv_image
        # Initiate ORB detector
        orb = cv2.ORB_create()
        # find the keypoints with ORB
        kp = orb.detect(img,None)
        # compute the descriptors with ORB
        kp, des = orb.compute(img, kp)
        # draw only keypoints location,not size and orientation
        img2 = cv2.drawKeypoints(img, kp, None, color=(0,255,0), flags=0)
        
        cv2.imshow('img',img2)
        cv2.waitKey(1)
def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
    	rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main(sys.argv)
