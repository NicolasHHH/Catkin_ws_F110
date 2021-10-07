#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

home = expanduser('~')
file_odom = open('waypoint4.csv', 'w') #strftime(home+'/rcws/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())
file_pose = open('waypoint5.csv', 'w') 
count = 0
count2 = 0

def pose_save_waypoint(data):
    quaternion = np.array([data.pose.orientation.x, 
                           data.pose.orientation.y, 
                           data.pose.orientation.z, 
                           data.pose.orientation.w])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    global count
    count += 1
    if(count>= 300):
    	count =0
    	print ("pose: ", data.pose.position.x,data.pose.position.y,euler[2])
    	file_pose.write('%f, %f, %f, %f\n' % (data.pose.position.x,data.pose.position.y,data.pose.orientation.z, data.pose.orientation.w))
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
    global count2
    count2+=1
    if (count2 >= 300):
    	count2=0
    	print ("odom:",data.pose.pose.position.x,data.pose.pose.position.y,euler[2])
    	file_odom.write('%f, %f, %f, %f, %f\n' % (data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w,speed))


   #speed
    #rospy.sleep(0.1)

def shutdown():
    file_odom.close()
    file_pose.close()
    print('Goodbye')
 
def listener(): 
    rospy.init_node('waypoints_logger', anonymous=True)
    rospy.Rate(2)
    rospy.Subscriber('/odom', Odometry, odom_save_waypoint,) # pf/pose/
    rospy.Subscriber('/gt_pose', PoseStamped, pose_save_waypoint) # pf/pose/
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
