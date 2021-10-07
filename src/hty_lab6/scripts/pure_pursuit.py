#!/usr/bin/env python
import rospy
import tf
from tf.transformations import euler_from_quaternion
import tf2_py

import tf2_ros


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
import rviz
import std_msgs.msg as std_msgs

import visualization_msgs.msg as viz_msgs
from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters
#from tf.transformations import euler_from_quaternion

VELOCITY = 2.0 # meters per second
LOOK_AHEAD_DIST = 2.5
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
        self.rviz_pub = rospy.Publisher('/waypoint_vis_array',viz_msgs.MarkerArray,queue_size = 2)
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
        aheadX,aheadY= x,y
        rotationZ = 0;
        mindistCar,mindistL = 60,40;
        stateclose = True
        for waypoint in self.waypoints:
            dist = self.dist_euclid(waypoint[0],waypoint[1],x,y)
            alpha = math.atan((waypoint[1]-y)/(waypoint[0]-x+0.0001))
            euler = euler_from_quaternion([0,0,waypoint[2],waypoint[3]])
            delta = abs(euler[2] - alpha)
            if(delta>3.14):
                delta -= 3.14
            if(delta<-3.14):
                delta += 3.14
            print("alpha",alpha,euler,delta)
            if (abs(dist-LOOK_AHEAD_DIST)<mindistL and delta < 1):
                #print(aheadX,aheadY,x,y,dist)
                mindistL = dist;
                aheadX = waypoint[0]
                aheadY = waypoint[1]
                rotationZ = waypoint[2]
                rotationW = waypoint[3]
                self.speed = waypoint[4]
        print("aheadXY: ",aheadX,aheadY,x,y)

        aheadPoint = PointStamped()
        aheadPoint.header.frame_id = "map"
        aheadPoint.point.x = aheadX
        aheadPoint.point.y = aheadY
        aheadPoint.point.z = 0

        # TODO: transform goal point to vehicle frame of refercence
        
        transformed_point = self.tf_listener.transformPoint("base_link",aheadPoint)
        
        # TODO: calculate curvature/steering angle
        curve = 2*(transformed_point.point.y)/LOOK_AHEAD_DIST**2
        angle = curve*0.5  #+3.14/4
        print(transformed_point.point.x,transformed_point.point.y,"angle",angle)

        
        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "pure" #"laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = VELOCITY

        if abs(angle) > 0.418:
            angle = angle/abs(angle)*0.418
        self.drive_pub.publish(drive_msg)
        rospy.Rate(1)
        return

    def odom_callback(self, odom_msg):
        #print("===odom_callback")
        self.mark_way_points()
        self.speed = odom_msg.twist.twist.linear.x;
        #rospy.sleep(0.1)
        return
    def mark_way_points(self):
        """
        Create slightly transparent disks for way-points.
        :param color: disk RGBA value
        """
        marker_array = []
        id = 0
        for wp in self.waypoints: 
            marker = Marker()
            marker.header.seq = 1
            id+=1
            marker.id = id
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE  # NOTE: color must be set here, not in rviz
            marker.action = Marker.ADD
            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = wp[0];
            marker.pose.position.y = wp[1];
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = wp[2];
            marker.pose.orientation.w = wp[3];
            marker.scale.x = 0.2;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 0.3; 
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker_array.append(marker)
        self.rviz_pub.publish( viz_msgs.MarkerArray(marker_array) );




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


            if(dist<mindistCar and stateclose):
                mindistCar = dist
                closeX = waypoint[0]
                closeY = waypoint[1]
                rotationZ = waypoint[2]
"""
