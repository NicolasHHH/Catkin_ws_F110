#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def odom_callback(data):
  global pub
  
  msg = PoseWithCovarianceStamped()
  msg.header = data.header
  msg.pose.pose = data.pose.pose
  msg.pose.covariance = data.pose.covariance
  pub.publish(msg)
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('fake_amcl_node')
        
    odom_topic = rospy.get_param('~odom_topic', 'ego_id/odom') 
    amcl_topic = rospy.get_param('~amcl_topic', '/amcl_pose')
    
    rospy.Subscriber(odom_topic, Odometry, odom_callback, queue_size=1)
    pub = rospy.Publisher(amcl_topic, AckermannDriveStamped, queue_size=1)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
