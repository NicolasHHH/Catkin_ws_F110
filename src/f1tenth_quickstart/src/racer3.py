#!/usr/bin/env python3
# pure pursuit
import rospy
from nav_msgs.msg import Odometry
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg 
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import tf

path_point = []

class pure_pursuit:

    def __init__(self):

        self.LOOKAHEAD_DISTANCE = 1.70#1.70 # meters
        self.VELOCITY = 5 # m/s
        #self.goal = 0
        self.goal_idx = 0
        self.velocity = 2.5#1.5
        self.tf_listener = tf.TransformListener()
        
        self.drive_pub = rospy.Publisher('/opp_id/drive', AckermannDriveStamped, queue_size = 10)
        rospy.Subscriber("/opp_id/odom", Odometry, self.callback, queue_size=1)
        
        self.read_waypoints()

    # Import waypoints.csv into a list (path_points)
    def read_waypoints(self):
        #rospack = rospkg.RosPack()
        #package_path=rospack.get_path('a_stars_pure_pursuit')
        #filename=package_path+'/waypoints/waypoints_1.csv'
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_w   = [float(point[2]) for point in path_points]
        #Initialize array of zeros with the same number of entries as the waypoint markers
        self.dist_arr= np.zeros(len(self.path_points_y))

    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Input data is PoseStamped message from topic /pf/viz/inferred_pose.
    # Runs pure pursuit and publishes velocity and steering angle.
    def callback(self,data):
        qx=data.pose.pose.orientation.x
        qy=data.pose.pose.orientation.y
        qz=data.pose.pose.orientation.z
        qw=data.pose.pose.orientation.w

        quaternion = (qx,qy,qz,qw)
        euler   = euler_from_quaternion(quaternion)
        yaw     = euler[2] 
        x = data.pose.pose.position.x # self.x
        y = data.pose.pose.position.y # self.y

        self.path_points_x = np.array(self.path_points_x)
        self.path_points_y = np.array(self.path_points_y)

        ## finding the distance of each way point from the current position 
        for i in range(len(self.path_points_x)):
            self.dist_arr[i] = self.dist((self.path_points_x[i],self.path_points_y[i]),(x,y))

        ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
        goal_arr = np.where((self.dist_arr < self.LOOKAHEAD_DISTANCE+0.3)&(self.dist_arr > self.LOOKAHEAD_DISTANCE-0.3))[0]
        # goal_arr : goal indexes
        
        # find those are ahead
        for idx in goal_arr:
            #line from the point position to the car position
            v1 = [self.path_points_x[idx]-x , self.path_points_y[idx]-y]
            #since the euler was specified in the order x,y,z the angle is wrt to x axis
            v2 = [np.cos(yaw), np.sin(yaw)]
            #find the angle between these two vectors NOTE:These are in world coordinates
            temp_angle = self.find_angle(v1,v2)
            if abs(temp_angle) < np.pi/2:
                self.goal_idx = idx
                break

        ##finding the distance of the goal point from the vehicle coordinator
        L = self.dist_arr[self.goal_idx]
        
        aheadPoint = PointStamped()
        aheadPoint.header.frame_id = "map"
        aheadPoint.point.x = self.path_points_x[self.goal_idx]
        aheadPoint.point.y = self.path_points_y[self.goal_idx]
        aheadPoint.point.z = 0
        # TODO: transform goal point to vehicle frame of refercence
        transformed_point = data
        transformed_point = self.tf_listener.transformPoint("base_link",aheadPoint)
        
        # TODO: calculate curvature/steering angle
        curve = 2*(transformed_point.point.y)/0.9**2
        angle = curve*0.4 #+3.14/4
        self.set_speed(angle)
        #self.const_speed(angle)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "pure" #"laser"
        drive_msg.drive.steering_angle = self.angle
        drive_msg.drive.speed = self.velocity

        self.drive_pub.publish(drive_msg)

    # USE THIS FUNCTION IF CHANGEABLE SPEED IS NEEDED
    def set_speed(self,angle):
        if (abs(angle)>0.2018):
            self.LOOKAHEAD_DISTANCE = 1.2
            # self.velocity = 1.5
            self.angle = angle

            if self.velocity - 1.5 >= 0.5:
                self.velocity -= 0.5#0.7

        else:
            self.LOOKAHEAD_DISTANCE = 1.2
            # self.velocity = 3.0
            self.angle = angle

            if self.VELOCITY - self.velocity > 0.2:
                self.velocity += 0.2
        print(angle,self.velocity)

    # USE THIS FUNCTION IF CONSTANT SPEED IS NEEDED
    def const_speed(self,angle):
        #self.LOOKAHEAD_DISTANCE = 2
        self.angle = angle
        self.velocity = 1.5#self.VELOCITY

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)


if __name__ == '__main__':
    with open("home/tianyang/f110_ws/waypoint6_100.csv" ,'r') as f: 
        cr = csv.reader(f)
        path_points = [tuple(line) for line in csv.reader(f)]
    print(path_points[0])

    rospy.init_node('pure_pursuit')
    C = pure_pursuit()  
    
    rospy.spin()