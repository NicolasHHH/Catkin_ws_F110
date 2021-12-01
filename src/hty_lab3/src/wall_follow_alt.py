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
DESIRED_DISTANCE_RIGHT = 0.6 # meters
DESIRED_DISTANCE_LEFT = 1.0
VELOCITY = 6 # meters per second
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters
AHEAD = 1.0

#ajout
#MODIF WALL FOLLOW PARAMS
T_TRANS = 6/VELOCITY  # 2.0
PROF_SUR = 2.0
ANG_SUR = 10

class WallFollow:

    def __init__(self):
        global prev_time
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        prev_time = rospy.get_time()
        
        #ajout
        self.b_trans = False
        self.t_trans = 0.0
        self.n_mur = -1
        

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
        global integral, prev_error, prev_time
        global kp,ki,kd
        angle = 0.0
        current_time = rospy.get_time()
        del_time = current_time - prev_time

        #ajout
        if self.b_trans:
            self.t_trans += del_time

        #TODO: Use kp, ki & kd to implement a PID controller for
        integral += prev_error * del_time
        angle = kp * error + ki * integral + kd * (error - prev_error) / del_time

        prev_error = error
        prev_time = current_time

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser" #"laser"
        drive_msg.drive.steering_angle = -angle * self.n_mur

        if abs(angle) > math.radians(0) and abs(angle) <= math.radians(10):
            #drive_msg.drive.acceleration = 0.3
            drive_msg.drive.speed = velocity*1.0

        elif abs(angle) > math.radians(10) and abs (angle) <= math.radians(20):
            #drive_msg.drive.acceleration = 0.1
            drive_msg.drive.speed = velocity*0.8
        else:
            drive_msg.drive.speed = velocity*0.6
            #drive_msg.drive.acceleration = -0.1
        print(drive_msg)
        self.drive_pub.publish(drive_msg)

    #ajout
    def sur(self, data, mur):
        dist = 0
        ang = (135 if mur == 1 else -45)
        for _ in range(85//ANG_SUR):
            ang += ANG_SUR 
            dist2 = self.getRange(data, ang)
            if abs(dist2 - dist) > PROF_SUR:
                return False
            dist = dist2
        return True

    #modif
    def follow(self, data, desired):
        
        front_angle = 90 + 35 * self.n_mur
        back_angle = 90 + 95 * self.n_mur  #185 if mur == 1 else -5 

        theta = math.radians(abs(front_angle - back_angle))

        front_dist = self.getRange(data, front_angle)
        back_dist = self.getRange(data, back_angle)
        
        alpha = math.atan2(front_dist * math.cos(theta) - back_dist,
                            front_dist * math.sin(theta))
            
        wall_dist = back_dist * math.cos(alpha)
        ahead_wall_dist = wall_dist + CAR_LENGTH * math.sin(alpha)*AHEAD

        return desired - ahead_wall_dist

    #modif
    def lidar_callback(self, data):
        global integral, prev_error, prev_time
        if not(self.b_trans) and not(self.sur(data.ranges, self.n_mur)):# and self.sur(data.ranges, -self.n_mur):
            self.b_trans = True
            self.n_mur *= -1
            prev_error = 0.0
            integral = 0.0
            prev_time = 0.0
        elif self.b_trans and self.t_trans > T_TRANS:
            self.b_trans = False
            self.t_trans = 0.0
            if self.sur(data.ranges,-self.n_mur) :
                prev_error = 0.0
                integral = 0.0
                prev_time = 0.0
                self.n_mur *= -1
        error = self.follow(data.ranges, DESIRED_DISTANCE_LEFT*(1+self.b_trans))

        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
        main(sys.argv)

