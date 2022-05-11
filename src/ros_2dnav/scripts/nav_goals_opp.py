#!/usr/bin/env python
from telnetlib import STATUS
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
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
  
def scan_callback(data):
      global status
      global car_x,car_y
      r = rospy.Rate(1) 
      fleshes = [(-0.771, -0.283, 0.098, 0.995),
             (7.962, -9.274, -0.828, 0.560),
             (2.207, -18.244, 0.776, 0.630)]
      if dist(car_x,car_y,fleshes[status][0],fleshes[status][1],4) :
          print(status)
          status = (status+1)%3;
          print(status)
      x = fleshes[status][0]
      y = fleshes[status][1]
      z = fleshes[status][2]
      w = fleshes[status][3]
      msg = PoseStamped()
      msg.header.stamp = rospy.Time.now()
      msg.header.frame_id = "map" 
      msg.header.seq = 1
      msg.pose.position.x = x
      msg.pose.position.y = y
      msg.pose.position.z =0.0
      msg.pose.orientation.x =0.0
      msg.pose.orientation.y =0.0
      msg.pose.orientation.z = z
      msg.pose.orientation.w = w
      pub.publish(msg)
      r.sleep()
      return
  
  

if __name__ == '__main__': 
  
  try:
    rospy.init_node('nav_goals_publisher_oppp')
    rospy.Subscriber("opp_id/odom", Odometry, odom_callback, queue_size=1)
    rospy.Subscriber("opp_id/scan", LaserScan, scan_callback, queue_size=1)
    # amcl_pose 
    # geometry_msgs/PoseWithCovariance pose 
    # geometry_msgs/Pose pose
    # position
    #rospy.Subscriber("/amcl_pose", PoseWithCovariance, amcl_callback, queue_size=1)
    pub = rospy.Publisher("/opp_id/move_base_simple/goal", PoseStamped, queue_size=1)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
