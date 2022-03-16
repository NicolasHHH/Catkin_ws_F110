#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

FREQUENCY = 10
count = 0
WAYPOINTS = []

class Global_planner():
    def __init__(self) -> None:
        return


def odom_save_waypoint(data):
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)
    #if data.twist.twist.linear.x>0.:
    global count
    global WAYPOINTS
    count +=1
    if (count >= FREQUENCY and is_not_in(data.pose.pose.position.x, data.pose.pose.position.y)):
        count=0
        WAYPOINTS.append((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z,data.pose.pose.orientation.w,speed))
    return

def is_not_in(x,y):
    for point in WAYPOINTS:
        if (point[0]-x)**2 + (point[1]-y)**2 <= 0.1:
            print("similar")
            return False
    return True


def shutdown():
    global WAYPOINTS
    print()
    print(len(WAYPOINTS),' waypoints Saved')
    file_odom = open('waypoints_berlin_teb_8_2_2_1_08.csv', 'w') 
    for point in WAYPOINTS:
        file_odom.write('%f, %f, %f, %f, %f\n' % point)
    file_odom.close()
    print('Goodbye')
 
def listener(): 
    rospy.init_node('waypoints_logger', anonymous=True)
    rospy.Rate(10)
    rospy.Subscriber('/odom', Odometry, odom_save_waypoint,) # pf/pose/
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
