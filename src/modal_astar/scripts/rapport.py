#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class Itinary(object):
    def __init__(self):
    
        rospy.Subscriber("/odom",Odometry, self.pose_callback, queue_size=1)
        self.iti_pub = rospy.Publisher("iti_marker", Marker, queue_size=2)
        
        self.LineStrip = Marker()
        self.LineStrip.header.frame_id = 'map'
        self.LineStrip.header.stamp = rospy.Time.now()
        self.LineStrip.ns = 'points_and_lines'
        self.LineStrip.pose.orientation.w = 1.0
        self.LineStrip.action = Marker.ADD
        self.LineStrip.id = 1
        self.LineStrip.type = Marker.LINE_STRIP
        self.LineStrip.color.a = 0.7
        self.LineStrip.scale.x = 0.03
        self.LineStrip.color.g = 0.8
        self.x,self.y = 0,0
        self.initialize = False
        return
        
    def pose_callback(self,pose_msg):
        self.initialize = True
        self.x,self.y= pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y
        return 
    
    def publish(self, event=None):
        if self.initialize:
            p = Point(self.x,self.y,0)
            self.LineStrip.points.append(p)
            if len(self.LineStrip.points)>500:
                self.LineStrip.points.pop(0)

            self.iti_pub.publish(self.LineStrip)

        p = Point(self.x,self.y,0)
        self.LineStrip.points.append(p)
        self.iti_pub.publish(self.LineStrip)
        return
    


def main(args):
     
    rospy.init_node("itinary")
    ab = Itinary()
    rospy.Timer(rospy.Duration(1.0/8.0), ab.publish)
    rospy.spin()

if __name__=='__main__':
        main(sys.argv)
