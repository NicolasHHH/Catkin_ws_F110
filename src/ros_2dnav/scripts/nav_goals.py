#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped



def scan_callback(data):
  msg = PoseStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = "map" 
  msg.header.seq = 1
  msg.pose.position.x =5.826568603515625
  msg.pose.position.y =-16.075592041015625
  msg.pose.position.z =0.0
  msg.pose.orientation.x =0.0
  msg.pose.orientation.y =0.0
  msg.pose.orientation.z =-0.618
  msg.pose.orientation.w =0.786
  pub.publish(msg)
  
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('nav_goals_publisher')
    
    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
