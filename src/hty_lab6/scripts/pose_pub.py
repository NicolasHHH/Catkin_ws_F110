





import rospy
import csv

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    waypoints = []
    with open('waypoint2.csv' ,'r') as f: 
            cr = csv.reader(f)
            for row in cr:
                array =[]
                for col in row:
                    array.append(eval(col))
                waypoints.append(array)
    print(waypoints[0])



    pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size = 2)
    
    while not rospy.is_shutdown():


        rospy.sleep()