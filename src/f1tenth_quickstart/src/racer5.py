#!/usr/bin/env python

import sys
import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#PID CONTROL PARAMS
kp = 1.0
kd = 0.5
ki = 0.0

servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

VELOCITY = 2.5
ANGLE_RANGE = 270
DESIRED_DISTANCE_RIGHT = 1.9
DESIRED_DISTANCE_LEFT = 1.9
CAR_LENGTH = 0.50

class Ant_Balance:

    def __init__(self):
        global prev_time
        #Topics & Subs, Pubs
        lidarscan_topic = '/ego_id/scan'
        drive_topic = '/ego_id/drive'
        prev_time = rospy.get_time()
        
        self.speed = VELOCITY
        self.left = [2]*10
        self.right = [2]*10
        
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10)

    def getRange(self, dis, alpha):
        return dis * math.cos(alpha) + self.speed*math.sin(alpha);

    def pid_control(self, error):
        global integral, prev_error,prev_time
        global kp,ki,kd
        
        integral += error;
        angle = kp * error + kd * (error - prev_error) + ki * integral;
        prev_error = error;

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser" #"laser"
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.speed = self.speed
        if abs(angle) > math.radians(10) and abs (angle) <= math.radians(20):
            #drive_msg.drive.acceleration = 0.1
            drive_msg.drive.speed = self.speed*0.8
            
        if abs(angle) > math.radians(20) and abs (angle) <= math.radians(30):
            #drive_msg.drive.acceleration = 0.1
            drive_msg.drive.speed = self.speed*0.6
        #print(drive_msg)
        self.drive_pub.publish(drive_msg)
    

    def lidar_callback(self, scan_msg):
        angle = scan_msg.angle_min
        angle_incre = scan_msg.angle_increment
        
        dis= scan_msg.ranges;
        
        angle_b, angle_a = angle+810*angle_incre ,angle+660*angle_incre
        dis_b, dis_a = dis[810],dis[660]
        angle_d, angle_c = angle+270*angle_incre ,angle+420*angle_incre
        dis_d, dis_c = dis[270],dis[420]
        
        
        alpha = math.atan((dis_a*math.cos(angle_a-angle_b)-dis_b)/(dis_a*math.sin(angle_a-angle_b)))
        beta = math.atan((dis_c*math.cos(angle_c-angle_d)-dis_d)/(dis_c*math.sin(angle_c-angle_d)))
        #print(alpha,beta)
        leftDist = self.getRange(dis_b,-alpha)
        rightDist = self.getRange(dis_d,beta)
        print(leftDist,rightDist)
        self.left.append(leftDist)
        self.left.pop(0)
        self.right.append(rightDist)
        self.right.pop(0)
        
        if(dis_b>4 and dis_d<4):error = -2
        elif (dis_b<4 and dis_d>4): error = 2
        else : error = np.mean(self.right)-np.mean(self.left)
            
        self.pid_control(error)
        return 
            
def main(args):
     
    rospy.init_node("AntBlance_node", anonymous=True)
    ab = Ant_Balance()
    rospy.spin()
    #np.savetxt('LaserScan.csv', np.array(LOG), delimiter=',')

if __name__=='__main__':
        main(sys.argv)