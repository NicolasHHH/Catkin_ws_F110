#!/usr/bin/env python
from telnetlib import STATUS
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance

  
def goal_callback(data):
      r = rospy.Rate(10) 
      msg = PoseStamped()
      msg.header.stamp = rospy.Time.now()
      msg.header.frame_id = "map" 
      msg.header.seq = 1
      msg.pose.position.x = data.pose.position.x
      msg.pose.position.y = data.pose.position.y
      msg.pose.position.z =0.0
      msg.pose.orientation.x =0.0
      msg.pose.orientation.y =0.0
      msg.pose.orientation.z = msg.pose.orientation.z 
      msg.pose.orientation.w = msg.pose.orientation.w
      pub.publish(msg)
      r.sleep()
      return
  
  

if __name__ == '__main__': 
  
  try:
    rospy.init_node('goal_to_goal')
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback, queue_size=1)
    pub = rospy.Publisher("/move_base_simple/current_goal", PoseStamped, queue_size=1)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
