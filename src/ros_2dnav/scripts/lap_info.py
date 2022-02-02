#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance



global status
status = 0
global car_x,car_y
car_x,car_y = 0,0


def dist(x,y,a,b,seuil):
      return ((x-a)**2 + (y-b)**2) < seuil**2

def odom_callback(data):
  global car_x,car_y
  car_x, car_y = data.pose.pose.position.x,data.pose.pose.position.y
  return


if __name__ == '__main__': 
  
  try:
    rospy.init_node('nav_goals_publisher')
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)
    rospy.spin()
    
    print("ros end")
    
    
  except rospy.ROSInterruptException:
    pass
