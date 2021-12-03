#!/usr/bin/python

#/cmd_vel  Type: geometry_msgs/Twist
#/odom     Type: nav_msgs/Odometry
#/scan     Type: sensor_msgs/LaserScan
# geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Turtle(object):

    def __init__(self):
    
        
        # recieve odom and pose msg
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.pose_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 2)
        
        
    def scan_callback(self,scan_msg):
        command = Twist()
        command.linear.x=0.3
        command.angular.x = 0.1
        self.move_pub.publish(command)
        return 
    
    def odom_callback(self,pose_msg):
        
        return 
def main():
    rospy.init_node('turtle control')
    turtle = Turtle()
    rospy.spin()

if __name__ == '__main__':
    main()
    
