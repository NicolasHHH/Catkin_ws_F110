#!/usr/bin/env python3
from __future__ import print_function
from numbers import Integral
import sys
import math
import numpy as np
from numpy.core.fromnumeric import amax, size
from numpy.core.getlimits import _KNOWN_TYPES
from numpy.ma.core import flatten_structured_array

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#Global Variables
detect_angle = 120
max_detect_dist = 5.25
past_ranges_t1 = past_ranges_t2 = past_ranges_t3 = past_ranges_t4 = current_ranges_t5 = np.zeros(int(1080 * detect_angle / 360))
width_car = 0.2032

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        prev_time = rospy.get_time()

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Two approaches include:
            1.Setting each value to the rolling mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        global past_ranges_t1, past_ranges_t2, past_ranges_t3, past_ranges_t4, current_ranges_t5, max_detect_dist
        
        #Store past laser data
        past_ranges_t1 = past_ranges_t2
        past_ranges_t2 = past_ranges_t3
        past_ranges_t3 = past_ranges_t4
        past_ranges_t4 = current_ranges_t5
        current_ranges_t5 = ranges
        
        # Get weighted rolling mean among 5 frames
        proc_ranges = 0.5 * current_ranges_t5 + 0.25 * past_ranges_t4 + 0.15 * past_ranges_t3 + 0.07 * past_ranges_t2 + 0.03 * past_ranges_t1
        
        # Rejecting high values
        proc_ranges[proc_ranges > max_detect_dist] = max_detect_dist
        return proc_ranges

    def set_bubble(self, proc_ranges, angle_incre):
        """ Set a bubble in which its center is the closest point to LiDAR and eliminate all points inside the bubble
        """
        #Find closest point to LiDAR
        closest_point = np.argmin(proc_ranges)

        #Eliminate all points inside 'bubble' (set them to zero) using consine rule
        bubble_radius = 3.75
        for i in range(size(proc_ranges)):
            if math.sqrt(proc_ranges[i] ** 2 + proc_ranges[closest_point] ** 2 - 2 * proc_ranges[i] * proc_ranges[closest_point] * math.cos(angle_incre)) < bubble_radius:
                proc_ranges[i] = 0   
        
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        index_list = [0 ,0]
        max_gap_value = 0
        start = True

        # Retrieve all start indeces and end indeces for the gap, the gap or free spaces refers to non-zero points
        for i in range(size(free_space_ranges)):
            if free_space_ranges[i] != 0 and start == True:
                index_list.append(i)
                start = False
            elif free_space_ranges[i] == 0 and start == False:
                index_list.append(i-1)
                start = True

        # Add in the last index if end index have not append into the list above (in case the last element is not zero)
        if len(index_list) % 2 == 1:
            index_list.append(size(free_space_ranges) - 1)

        # Compute the max gap start and end indeces
        max_gap_index = [index_list[0], index_list[1]]
        for i in range(int(len(index_list) / 2)):
                gap_value = index_list[2 * i + 1] - index_list[2 * i]
                if abs(gap_value) >= max_gap_value:
                    max_gap_value = gap_value
                    max_gap_index = [index_list[2 * i], index_list[2 * i +1]]
                
        return max_gap_index
    
    def find_best_point(self, start_i, end_i, ranges, angle_incre):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
        """

        disparity_dist = 2.5    # Minimum distance between consecutive points which consider as disparities
        meaningful_ranges = ranges[start_i: end_i+2]  # Only concern about the data inside maximum gap

        # Extending disparities, reference: https://www.youtube.com/watch?v=ctTJHueaTcY
        for i in range(len(meaningful_ranges)):
            if( i < len(meaningful_ranges) - 1):

                # Case 1: Detect Right side disparities
                if meaningful_ranges[i+1] - meaningful_ranges[i] > disparity_dist:

                    # Add 50 counts for more tolerance (* Can be improve)
                    steps_to_skip = disparity_count =  (width_car) // (meaningful_ranges[i] * angle_incre) + 50
                   
                    a = 1
                    # Extend disparities
                    while(disparity_count > 0 and i + a < len(meaningful_ranges)):
                        meaningful_ranges[i + a] = meaningful_ranges[i]     
                        disparity_count -= 1
                        a += 1

                    # Skips those extended disparity points
                    i += steps_to_skip - 1     
                    continue

                # Case 2: Detect Left side disparities
                elif meaningful_ranges[i] - meaningful_ranges[i+1] > disparity_dist:

                    # Add 50 counts for more tolerance (* Can be improve)
                    if(meaningful_ranges[i+1] != 0):
                        steps_to_skip = disparity_count = (width_car) // (meaningful_ranges[i+1] * angle_incre) + 50
                    else:
                        steps_to_skip = disparity_count = 0
                    a = 0

                    # Add 50 counts for more tolerance (* Can be improve)
                    while(disparity_count > 0 and i - a >= 0 and i + 1 <len(meaningful_ranges)):
                        meaningful_ranges[i + a] = meaningful_ranges[i+1]
                        disparity_count -= 1
                        a -= 1

                    # Skips those extended disparity points
                    i += steps_to_skip - 1   
                    continue

        # Get max distances and the its index (if multiple same max distance, the index will be the index of first max distance)
        max_dist = np.amax(meaningful_ranges)
        max_dist_index = np.argmax(meaningful_ranges)

        # Compute same maximum distances data occurances 
        max_dist_occurances = 0
        sliced_ranges = meaningful_ranges[max_dist_index:]
        for i in range(len(sliced_ranges) - 1):
            if sliced_ranges[i+1] == max_dist:
                max_dist_occurances +=1
            elif sliced_ranges[i+1] != max_dist:
                break

        # Return the middle index of the multiple max distance data
        return np.argmax(meaningful_ranges) + start_i +  int ((max_dist_occurances + 1) / 2)
        

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        # Retrieve range and angle increment data
        ranges = np.asarray(data.ranges)
        angle_incre = data.angle_increment

        # Filter unwanted ranges data
        ranges = ranges[int(0.5 * size(ranges) - size(ranges) * detect_angle/ (2 * 360)): int(0.5 * size(ranges) + size(ranges) * detect_angle/ (2 * 360))]
        proc_ranges = self.preprocess_lidar(ranges)

        # Eliminate points within bubble
        proc_ranges = self.set_bubble(proc_ranges, angle_incre)

        # Find max length gap 
        max_gap_indeces = self.find_max_gap(proc_ranges)
        start_i = max_gap_indeces[0]
        end_i = max_gap_indeces[1]

        # Find the best point in the gap
        best_point_index = self.find_best_point(start_i, end_i, proc_ranges, angle_incre)

        # Find desire angle
        angle = (best_point_index - (len(ranges) - 1) / 2) * angle_incre

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"

        if abs(angle) < math.radians(0.5):
            drive_msg.drive.steering_angle = 0
        else:
            drive_msg.drive.steering_angle = angle
        if abs(angle) > math.radians(0) and abs(angle) <= math.radians(3):
            drive_msg.drive.speed = 6.3
        elif abs(angle) > math.radians(3) and abs (angle) <= math.radians(10):
            drive_msg.drive.speed = 5.0
        elif abs(angle) > math.radians(10) and abs (angle) <= math.radians(20):
            drive_msg.drive.speed = 3.5
        else:
            drive_msg.drive.speed = 1.5

        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
