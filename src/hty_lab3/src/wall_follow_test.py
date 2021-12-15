#!/usr/bin/env python3

import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#PID CONTROL PARAMS
kp = 1.0
kd = 0.
ki = 0


servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
VELOCITY = 2.50


ANGLE_RANGE = 270
DESIRED_DISTANCE_RIGHT = 1.9
DESIRED_DISTANCE_LEFT = 1.9
CAR_LENGTH = 0.50

LOG = [[0]*11]

class WallFollow:

    def __init__(self):
        global prev_time
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        prev_time = rospy.get_time()
        
        self.speed = VELOCITY
        
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10)

    def getRange(self, dis_b, alpha):
        return dis_b * math.cos(alpha) + self.speed*math.sin(alpha);

    def pid_control(self, error,dis):
        global integral, prev_error,prev_time
        global kp,ki,kd
        
        integral = prev_error + error;
        angle = kp * error + kd * (error - prev_error) + ki * integral;
        prev_error = error;

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser" #"laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.speed
        if abs(angle) > math.radians(10) and abs (angle) <= math.radians(20):
            #drive_msg.drive.acceleration = 0.1
            drive_msg.drive.speed = self.speed*0.8
        #print(drive_msg)
        
        
        log = list(dis[0:10])+[angle]
        #LOG.append(log)
        self.drive_pub.publish(drive_msg)

    def followLeft(self, leftDist):
        return DESIRED_DISTANCE_LEFT - leftDist
    

    def lidar_callback(self, scan_msg):
        angle = scan_msg.angle_min
        angle_incre = scan_msg.angle_increment
        
        dis= scan_msg.ranges;
        dis_a, dis_b, angle_a, angle_b = 0,0,0,0
        
        flag = 0
        for i in range(0,len(dis),10):
            if angle > -1.57 and angle < -1.4 and flag == 0:
                dis_b = dis[i];
                flag = 1;
                angle_b = angle;
                
            if angle > -1.0 and angle < -0.8 and flag == 1:
                dis_a = dis[i];
                flag = 2;
                angle_a = angle;
                
            angle += angle_incre*10;

            
        alpha = math.atan((dis_a*math.cos(angle_a-angle_b)-dis_b)/(dis_a*math.sin(angle_a-angle_b)))
        leftDist = self.getRange(dis_b,alpha);
        error = self.followLeft(leftDist);
        self.pid_control(error,dis)
        return 
            
def main(args):
    
        
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()
    #np.savetxt('LaserScan.csv', np.array(LOG), delimiter=',')

if __name__=='__main__':
        main(sys.argv)
