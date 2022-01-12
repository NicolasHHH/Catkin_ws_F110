#!/usr/bin/env python3

import sys
import rospy
import math
import numpy as np

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class Itinary():
    def __init__(self):
    
        rospy.Subscriber("/gt_pose", PoseStamped, self.pose_callback, queue_size=1)
        self.iti_pub = rospy.Publisher("iti_marker", Marker, queue_size=2)
        
        self.LineStrip = Marker()
        self.LineStrip.header.frame_id = 'map'
        self.LineStrip.header.stamp = rospy.Time.now()
        self.LineStrip.ns = 'points_and_lines'
        self.LineStrip.pose.orientation.w = 1.0
        self.LineStrip.action = Marker.ADD
        self.LineStrip.id = 1
        self.LineStrip.type = Marker.LINE_STRIP
        self.LineStrip.color.a = 1.0
        self.LineStrip.scale.x = 0.03
        self.LineStrip.color.g = 0.8
        
        return
    
    def pose_callback(self,pose_msg):
        
        mx,my = pose_msg.pose.position.x,pose_msg.pose.position.y
        p = Point(mx,my,0)
        self.LineStrip.points.append(p)
        self.iti_pub.publish(self.LineStrip)
        return
    


def main(args):
     
    rospy.init_node("itinary", anonymous=True)
    ab = Itinary()
    rospy.spin()

if __name__=='__main__':
        main(sys.argv)
