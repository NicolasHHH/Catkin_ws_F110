#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#PID CONTROL PARAMS
kp = 1.0
kd = 0.001
ki = 0.005
servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0
prev_time = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 1.3 # meters
DESIRED_DISTANCE_LEFT = 1
VELOCITY = 3 # meters per second
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters
AHEAD = 1.0

global shut
shut = 0

class WallFollow:

    def __init__(self):
        global prev_time
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        prev_time = rospy.get_time()
        
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length at angle alpha
        #make sure to take care of nans etc.
        #TODO: implement
        if angle >= -45 and angle <= 225:
            iterator = len(data) * (angle + 90) / 360
            if not np.isnan(data[int(iterator)]) and not np.isinf(data[int(iterator)]):
                return data[int(iterator)]

    def pid_control(self, error, velocity):
        global integral, prev_error,prev_time
        global kp,ki,kd
        angle = 0.0
        current_time = rospy.get_time()
        del_time = current_time - prev_time
        #TODO: Use kp, ki & kd to implement a PID controller for
        integral += prev_error * del_time
        angle = kp * error + ki * integral + kd * (error - prev_error) / del_time

        prev_error = error
        prev_time = current_time

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser" #"laser"
        drive_msg.drive.steering_angle = -angle

        if abs(angle) > math.radians(0) and abs(angle) <= math.radians(10):
            #drive_msg.drive.acceleration = 0.3
            drive_msg.drive.speed = velocity*1.0

        elif abs(angle) > math.radians(10) and abs (angle) <= math.radians(20):
            #drive_msg.drive.acceleration = 0.1
            drive_msg.drive.speed = velocity*0.8
        else:
            drive_msg.drive.speed = velocity*0.6
            #drive_msg.drive.acceleration = -0.1
        #print(drive_msg)
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDesired):
        #Follow left wall as per the algorithm
        #TODO:implement
        right_front_angle = 55
        right_middle_angle = 20
        right_back_angle = -5
        left_front_angle = 125
        left_middle_angle = 160
        left_back_angle = 185


        theta = math.radians(abs(left_front_angle - left_back_angle))

        left_front_dist = self.getRange(data, left_front_angle)
        left_back_dist = self.getRange(data, left_back_angle)

        alpha = math.atan2(left_front_dist * math.cos(theta) - left_back_dist,
                            left_front_dist * math.sin(theta))
        wall_dist = left_back_dist * math.cos(alpha)
        ahead_wall_dist = wall_dist + CAR_LENGTH * math.sin(alpha)*AHEAD

        return leftDesired - ahead_wall_dist

    def lidar_callback(self, data):
        global shut
        shut += 1
        print ("shut: ", shut)
        if shut == 10:
            rospy.signal_shutdown("shutdown !!!")
        error = self.followLeft(data.ranges, DESIRED_DISTANCE_LEFT)
        #send error to pid_control
        self.pid_control(error, VELOCITY)
def main(args):
    for i in range(10):
        shut = 0
        rospy.init_node("WallFollow_node", anonymous=True)
        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        wf = WallFollow()
        rospy.sleep(0.1)
        rospy.spin()

if __name__=='__main__':
        main(sys.argv)
