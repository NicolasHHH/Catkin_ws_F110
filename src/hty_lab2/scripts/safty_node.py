#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool 
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np 



class Safety(object):
    """
    The class that handles emergeksskncy braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        # create ROS subscribers and publishers.
        self.speed = 0
        self.brake_pub = rospy.Publisher('brake', AckermannDriveStamped, queue_size=100)
        self.brake_bool_pub = rospy.Publisher('brake_bool', Bool, queue_size=100)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

    def odom_callback(self, odom_msg):
        # update current speed
        # x: speed of forward direction, +ve if forward, -ve if reversed
        self.speed = odom_msg.twist.twist.linear.x  
        rospy.loginfo(f"Odometry Message Received: {self.speed}")

    def scan_callback(self, scan_msg):
        # calculate TTC
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max 
        angle_inc = scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(angle_min, angle_max, angle_inc)

        # projected velocity
        v = self.speed * np.cos(angles)
        eps = 1e-5 # for numerical stability
        ttc = ranges / (np.maximum(v, 0) + eps)
        # print(f'min ttc: {np.min(ttc)}')

        # publish brake message and publish controller bool
        ttc_thres =  2
        perc_thres = 0.1 # requires 10% of ttc to be above ttc threshold to reduce false positive

        if np.sum(ttc < ttc_thres)/len(ttc) > perc_thres:
            print("BRAKE!")
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = 0.0
   
            self.brake_pub.publish(ack_msg)
            self.brake_bool_pub.publish(True)

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    main()
