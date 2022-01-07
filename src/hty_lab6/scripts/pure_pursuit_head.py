#!/usr/bin/env python3
import rospy
from rospy.core import rospyinfo 
import tf
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import csv

# ROS msg types and libraries
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped


import visualization_msgs.msg as viz_msgs
from visualization_msgs.msg import Marker

# static parameters
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters
VELOCITY = 3.5 # meters per second
HEADER_DIS = 1.0*CAR_LENGTH
global way_points 
way_points= []


class PurePursuit(object):

    def __init__(self):
   
        self.tf_listener = tf.TransformListener()
        
        # recieve odom and pose msg
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.pose_sub = rospy.Subscriber('/gt_pose', PoseStamped, self.pose_callback)
        
        # publish drive msg
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 2)
       
        self.rviz_pub = rospy.Publisher('/waypoint_vis_array',viz_msgs.MarkerArray,queue_size = 2)
        self.rviz_pub_point = rospy.Publisher('/waypoint_vis',viz_msgs.Marker,queue_size = 2)
        #self.frame = "."
        # self.time = 0.0
        #self.velocity = 0.0
        self.waypoints = way_points
        self.carX = 0 
        self.carY = 0
        self.carA = 0
        return
        
    def dist_euclid(self,x1,y1,x2,y2):
        return (x1-x2)**2 +(y1-y2)**2

    def pose_callback(self, pose_msg):
        
        #self.mark_way_points() # huge cost
        
        # calculate the header point
        # the virtual point that is ahead of the car for a distance of header_dis;
        
        #rospy.loginfo("euler: %f",euler[2])
        #rotationZ = self.waypoints[0][2]
        #rotationW = self.waypoints[0][3]
        #euler = euler_from_quaternion([0,0,z,w])
        targetX = self.carX + math.cos(self.carA)*HEADER_DIS
        targetY = self.carY + math.sin(self.carA)*HEADER_DIS
        NearX  = targetX
        NearY = targetY
        dis_Near_target = 100
        for waypoint in way_points:
            dist = self.dist_euclid(waypoint[0],waypoint[1],targetX,targetY)
            if dist < dis_Near_target:
                dis_Near_target = dist
                NearX = waypoint[0]
                NearY = waypoint[1]
                
        self.mark_point(NearX,NearY,0,0) #rotationZ,rotationW)

        aheadPoint = PointStamped()
        aheadPoint.header.frame_id = "map"
        aheadPoint.point.x = NearX
        aheadPoint.point.y = NearY
        aheadPoint.point.z = 0
        # TODO: transform goal point to vehicle frame of refercence
        transformed_point = pose_msg
        transformed_point = self.tf_listener.transformPoint("base_link",aheadPoint)
        
        # TODO: calculate curvature/steering angle
        curve = 2*(transformed_point.point.y)/0.9**2
        angle = curve*0.4 #+3.14/4
        #print(transformed_point.point.x,transformed_point.point.y,"angle",angle)

        
        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "pure" #"laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = VELOCITY

        if abs(angle) > 0.418:
            angle = angle/abs(angle)*0.418

        self.drive_pub.publish(drive_msg)
        #self.mark_way_points()
        rospy.sleep(0.01)
        return

    def odom_callback(self, odom_msg):
        #self.mark_way_points()
        self.speed = odom_msg.twist.twist.linear.x
        self.carX = odom_msg.pose.pose.position.x
        self.carY = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.orientation.z
        w = odom_msg.pose.pose.orientation.w
        euler = euler_from_quaternion([0,0,z,w])
        self.carA  = euler[2]
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
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = wp[2];
            marker.pose.orientation.w = wp[3];
            marker.scale.x = 0.2;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 0.8; 
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker_array.append(marker)
        self.rviz_pub.publish( viz_msgs.MarkerArray(marker_array) );
        
    def mark_point(self,aheadX,aheadY,aheadZ,aheadW):
        """
        Create slightly transparent disks for way-points.
        :param color: disk RGBA value
        """
        id = 0
        marker = Marker()
        marker.header.seq = 100
        marker.id = id
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE  # NOTE: color must be set here, not in rviz
        marker.action = Marker.ADD
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = aheadX;
        marker.pose.position.y = aheadY;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = aheadZ;
        marker.pose.orientation.w = aheadW;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 0.9; 
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        self.rviz_pub_point.publish( marker );
        return




def main():

    with open("waypoint6_100.csv" ,'r') as f: 
        cr = csv.reader(f)
        for row in cr:
                array =[]
                for col in row:
                    array.append(eval(col))
                way_points.append(array)
    print(way_points[0])
    
    
    #tf.listener()
    rospy.init_node('pure_pursuit_node')
    rospy.Rate(100)
    
    pp = PurePursuit()
    
    rospy.spin()
if __name__ == '__main__':
    main()
