#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class Itinary(object):
    def __init__(self):
    
        
        rospy.Subscriber("/opp_id/odom", Odometry, self.opp_callback, queue_size=1)
        rospy.Subscriber("/ego_id/odom", Odometry, self.ego_callback, queue_size=1)
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
        self.LineStrip.color.b = 0.8

        self.LineStrip2 = Marker()
        self.LineStrip2.header.frame_id = 'map'
        self.LineStrip2.header.stamp = rospy.Time.now()
        self.LineStrip2.ns = 'points_and_lines'
        self.LineStrip2.pose.orientation.w = 1.0
        self.LineStrip2.action = Marker.ADD
        self.LineStrip2.id = 2
        self.LineStrip2.type = Marker.LINE_STRIP
        self.LineStrip2.color.a = 1.0
        self.LineStrip2.scale.x = 0.03
        self.LineStrip2.color.r = 0.8
        self.LineStrip2.color.g = 0.3

        self.ego_x,self.ego_y,self.opp_x,self.opp_y= 0,0,0,0
        self.initialize_ego = False
        self.initialize_opp = False
        
        return
    
    def ego_callback(self,pose_msg):
        self.initialize_ego = True
        self.ego_x,self.ego_y= pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y
        return 
    
    def ego_publish(self, event=None):
        if self.initialize_opp and self.initialize_ego:
            p = Point(self.ego_x,self.ego_y,0)
            self.LineStrip.points.append(p)
            if len(self.LineStrip.points)>2000:
                self.LineStrip.points.pop(0)
            self.iti_pub.publish(self.LineStrip)
        return

    def opp_callback(self,pose_msg):
        self.initialize_opp = True
        self.opp_x,self.opp_y= pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y
        return 
    
    def opp_publish(self, event=None):
        # print("opp")
        if self.initialize_opp and self.initialize_ego:
            p = Point(self.opp_x,self.opp_y,0)
            self.LineStrip2.points.append(p)
            if len(self.LineStrip2.points)>2000:
                self.LineStrip2.points.pop(0)
            self.iti_pub.publish(self.LineStrip2)
        return



def main(args):
     
    rospy.init_node("itinary")
    ab = Itinary()
    rospy.Timer(rospy.Duration(1.0/10.0), ab.ego_publish)
    rospy.Timer(rospy.Duration(1.0/10.0), ab.opp_publish)
    rospy.spin()

if __name__=='__main__':
        main(sys.argv)
