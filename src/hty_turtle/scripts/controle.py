#!/usr/bin/python

#/cmd_vel  Type: geometry_msgs/Twist
#/odom     Type: nav_msgs/Odometry
#/scan     Type: sensor_msgs/LaserScan
# geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class Turtle(object):

    def __init__(self):
        
        
        # recieve odom and pose msg
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.pose_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 2)
        
        
    def scan_callback(self,scan_msg):
        
        obscured = 0;
        
        length = len(scan_msg.ranges)
        print(scan_msg.ranges[0],scan_msg.ranges[90],scan_msg.ranges[180],scan_msg.ranges[270])
        
        security = 0.5
                
        for i in range(0,length//12):
            if scan_msg.ranges[i] < security:
                obscured = 1
        for i in range(length//12*11,length):
            if scan_msg.ranges[i] < security:
                obscured = 1
        
        command = Twist()
        if obscured==0:
            print("clear")
            command.linear.x= 0.3
            command.angular.z= 0
        else :
            print("obstacle in close range")
            command.linear.x= 0
            command.angular.z= 0.6
        self.move_pub.publish(command)
        return 
    
    def odom_callback(self,pose_msg):
        
        return 
def main():
    rospy.init_node('turtle_control')
    turtle = Turtle()
    rospy.spin()

if __name__ == '__main__':
    main()
    
