#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback) #/rgb
    
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # convert ros Image into cv2 format 
        except CvBridgeError as e:
            print(e)

        # Image processing below
        (rows,cols,channels) = cv_image.shape # 720 1080
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (500,500), 50, (255,255,0))

        cv2.imshow("HTY window", cv_image)
        cv2.waitKey(3)  


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
