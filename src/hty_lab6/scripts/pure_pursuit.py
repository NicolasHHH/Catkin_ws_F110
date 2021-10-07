#!/usr/bin/env python
import rospy
import tf
from tf import listener
import tf2_ros
import tf_conversions

import math
import numpy as np
import csv

# traitement du fichier
import atexit

# TODO: import ROS msg types and libraries
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import visualization_msgs.msg as viz_msgs
import std_msgs.msg as std_msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters
#from tf.transformations import euler_from_quaternion

VELOCITY = 2.0 # meters per second
LOOK_AHEAD_DIST = 0.8
global way_points 
way_points= []



class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.waypoints = way_points
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.pose_sub = rospy.Subscriber('/gt_pose', PoseStamped, self.pose_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 2)
        self.frame = ""
        self.time = 0.0
        self.velocity = 0.0
        # TODO: create ROS subscribers and publishers.
        return
    def dist_euclid(self,x1,y1,x2,y2):
        return math.sqrt((x1-x2)**2 +(y1-y2)**2)

    def pose_callback(self, pose_msg):
        print("===pose_callback")
        # TODO: find the current waypoint to track using methods mentioned in lecture
        self.frame = pose_msg.header.frame_id
        self.time = pose_msg.header.stamp
        x = pose_msg.pose.position.x;
        y = pose_msg.pose.position.y;
        z = pose_msg.pose.orientation.z;
        w = pose_msg.pose.orientation.w;
        print( "voiture at: "+str(x)+" "+str(y))#+" orient: "+str(z)+" "+str(w))

        closeX,closeY = x+100.0, y+100.0
        aheadX,aheadY= x+100,y+100
        rotationZ = 0;
        mindistCar,mindistL = 10,20;
        stateclose = True
        for waypoint in self.waypoints:
            dist = self.dist_euclid(waypoint[0],waypoint[1],x,y)
            if(dist<mindistCar and stateclose):
                mindistCar = dist
                closeX = waypoint[0]
                closeY = waypoint[1]
                rotationZ = waypoint[2]
            if (abs(dist-LOOK_AHEAD_DIST)<mindistL):
                #print(aheadX,aheadY,x,y,dist)
                mindistL = dist;
                aheadX = waypoint[0]
                aheadY = waypoint[1]
                rotationZ = waypoint[2]
                rotationW = waypoint[3]
                self.speed = waypoint[4]
        #print("aheadXY: ",aheadX,aheadY,rotationZ)

        aheadPoint = PointStamped()
        aheadPoint.header.frame_id = "map"
        aheadPoint.point.x = aheadX
        aheadPoint.point.y = aheadY
        aheadPoint.point.z = 0

        # TODO: transform goal point to vehicle frame of refercence
        
        transformed_point = self.tf_listener.transformPoint("base_link",aheadPoint)
        
        # TODO: calculate curvature/steering angle
        curve = 2*(transformed_point.point.y)/LOOK_AHEAD_DIST**2
        angle = curve*0.3  #+3.14/4
        print(aheadY,transformed_point.point.y,"angle",angle)

        
        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "pure" #"laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = VELOCITY

        if abs(angle) > 0.418:
            angle = angle/abs(angle)*0.418
        self.drive_pub.publish(drive_msg)
    
        return

    def odom_callback(self, odom_msg):
        #print("===odom_callback")
        self.speed = odom_msg.twist.twist.linear.x;
        return



def main():

    with open("waypoint4.csv" ,'r') as f: 
        cr = csv.reader(f)
        for row in cr:
                array =[]
                for col in row:
                    array.append(eval(col))
                way_points.append(array)
    print(way_points[0])
    #tf.listener()
    rospy.init_node('pure_pursuit_node')
    rospy.Rate(0.1)
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()


"""
 br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        if (stateclose):
            t.transform.translation.x = closeX
            t.transform.translation.y = closeY
        else:
            t.transform.translation.x = aheadX
            t.transform.translation.y = aheadY
        t.transform.translation.z = 0.0
        print(rotationZ)
        quaternion = tf.transformations.quaternion_from_euler(0,0,rotationZ)
        #type(pose) = geometry_msgs.msg.Pose
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        
        
        t.transform.rotation = tf.transformations.quaternion_from_euler(0,0,rotationZ,0)
        br.sendTransform(t)
        trans = self.tf_buffer.lookup_transform(target_frame="map", source_frame="base_link", time=rospy.Time(0))
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = aheadX
        t.transform.translation.y = aheadY
        t.transform.translation.z = 0.0
        #type(pose) = geometry_msgs.msg.Pose
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = rotationZ
        t.transform.rotation.w = rotationW
        
        br.sendTransform(t)
        trans = self.tf_buffer.lookup_transform(target_frame="map", source_frame="base_link", time=rospy.Time(0))

        #self.tf_publish = rospy.Publisher('/self_tf' , geometry_msgs.msg.Twist, queue_size=1)
        
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
"""
