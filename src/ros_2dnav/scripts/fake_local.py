#!/usr/bin/env python

import rospy, math
from nav_msgs.msg import Odometry


def odom_callback(data):
  global pub
  
  pub.publish(data)
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('fake_local_node')
    rospy.Subscriber("/odom" ,Odometry, odom_callback, queue_size=1)
    pub = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
