#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

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
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength=100,maxLineGap=10)
        if lines is not None :
            for line in lines:
                x1,y1,x2,y2 = line[0]
                cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
        cv2.imwrite('houghlines5.jpg',img)
        cv2.imshow('img',img)
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
