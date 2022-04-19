#!/usr/bin/python

import numpy as np
import math
import csv

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from tf2_ros import transform_listener
from tf.transformations import euler_from_quaternion
import tf


ARM_LENGHT = 0.5

# pure pursuit
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters
VELOCITY = 2.0 # meters per second
HEADER_DIS = 1.8*CAR_LENGTH 

global way_points 
way_points= []


# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self,x,y,parent_index,root=False):
        self.x = x
        self.y = y
        self.parent = parent_index
        self.is_root = root

# class def for RRT
class RRT(object):
    def __init__(self):
        
        self.map_OK = False
        self.map_height, self.map_width = 600,600
        self.map = OccupancyGrid()
        
        self.x,self.y = 0 ,0
        self.oz,self.ow = 0,0
        self.vision =6.0
        self.tree = []
        self.goal_x =0
        self.goal_y =0
        
        # pure pursuit
        self.waypoints = way_points
        self.carA = 0
        self.path = []
        self.tf_listener = tf.TransformListener()
        
        # subscribers
        rospy.Subscriber("map",OccupancyGrid,self.read_map, queue_size=1)

        # publishers
        self.drive_pub = rospy.Publisher("drive", AckermannDriveStamped, queue_size = 2)
        self.occ_pub = rospy.Publisher("occup", OccupancyGrid, queue_size = 2)
        
        self.iti_pub = rospy.Publisher("itin_marker", Marker, queue_size=20)
        self.tree_pub = rospy.Publisher("tree_marker", MarkerArray, queue_size=20)
        self.waypoint_pub = rospy.Publisher('waypoint_vis',Marker,queue_size = 2)
        
    
    def read_map(self,map_data):
        if self.map_OK == False:
            self.map.info = map_data.info
            self.map.header = map_data.header
            self.map.data = map_data.data
            self.map_OK = True 
            print("mapok")
            #rospy.Subscriber("odom", PoseStamped, self.pf_callback,queue_size=1)
            #rospy.Subscriber("odom", PoseStamped, self.path_callback,queue_size=1)
            #rospy.Subscriber("scan", LaserScan, self.scan_callback,queue_size=1)
        print("mapok")
        self.occ_pub.publish(self.map)
        print(type(tuple(list(self.map.data))))
        
        
        return
        
    def ind_2_rc(self,ind):
        row = int(ind//self.map.info.width)
        col = ind%self.map.info.width - 1
        return row,col
    
    def rc_2_ind(self, r,  c):
        return r*self.map.info.width + c

    def coord_2_ind(self,x, y):
        r = (y-self.map.info.origin.position.y)/self.map.info.resolution
        c = (x-self.map.info.origin.position.x)/self.map.info.resolution
        return rc_2_ind(r,c)
    
    def ind_2_coord(self,ind):
        row,col = self.ind_2_rc(ind)
        y = r*self.map.info.resolution+self.map.info.origin.position.y
        x = r*self.map.info.resolution+self.map.info.origin.position.x
        return x,y
    
    
    
    def car_map(self,cx,cy):
        mx = cx*math.cos(self.carA)-cy*math.sin(self.carA)+self.x
        my = cx*math.sin(self.carA)+cy*math.cos(self.carA)+self.y
        return [mx,my]
    
    def map_car(self,mx,my):
        cx = (mx-self.x)*math.cos(-self.carA)-(my-self.y)*math.sin(-self.carA)
        cy = (mx-self.x)*math.sin(-self.carA)+(my-self.y)*math.cos(-self.carA)
        return [cx,cy]
    
    def dist_euclid(self,x1,y1,x2,y2):
        return (x1-x2)**2 +(y1-y2)**2
    
    
    def update_occupancy(self,rg,angle):
        cx = math.cos(angle)*rg
        cy = math.sin(angle)*rg
        gx,gy = self.cood_grid(cx,cy)
        if (self.extra//2<=gx and gx<self.award+self.extra//2 and gy>=self.extra//2  and gy<self.expand+self.extra//2 ):
            self.occ_grid[gx+1][gy+1] = 1
            self.occ_grid[gx][gy] = 1
            self.occ_grid[gx-1][gy-1] = 1
            self.occ_grid[gx-1][gy+1] = 1
            self.occ_grid[gx+1][gy-1] = 1
            self.occ_grid[gx+2][gy] = 1
        return 
        
    def scan_callback(self, scan_msg):
        
        print("scan")

        length = len(scan_msg.ranges)
        step = 8
        intercept = 150
        incre_angle = scan_msg.angle_increment
        min_angle = scan_msg.angle_min 
        for i in range(intercept,length-intercept,step):
            rg = scan_msg.ranges[i]
            if rg > self.vision :  
                continue
            angle = min_angle+incre_angle*i
            self.update_occupancy(rg,angle)
        
        # root 
        self.tree = []
        self.tree.append(Node(0,0,0,True)) # in car frame
        latest_node = self.tree[0]
        i=0
        
        while self.is_goal(latest_node) == False:
            i+=1
            sample_point = self.sample()
            nearest_node_index = self.nearest(sample_point)
            new_node = self.steer(nearest_node_index,sample_point)
            
            #self.set_sample(new_node.x,new_node.y)
            #self.red_pub.publish(self.samples)
            if self.check_collision(self.tree[nearest_node_index],new_node):
                latest_node = new_node
                self.tree.append(new_node)
            print("iteration : ",i)
            if(i>200) : break
        
        self.path = self.find_path(latest_node) 
        return
        
    def pf_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        """
        
        self.x = pose_msg.pose.position.x
        self.y = pose_msg.pose.position.y
        self.oz = pose_msg.pose.orientation.z
        self.ow = pose_msg.pose.orientation.w
        euler = euler_from_quaternion([0,0,self.oz,self.ow])
        self.carA  = euler[2]
        #print("pose:",self.x,self.y,self.carA)
        
        targetX = self.x + math.cos(self.carA)*HEADER_DIS
        targetY = self.y + math.sin(self.carA)*HEADER_DIS
        NearX  = targetX
        NearY = targetY
        dis_Near_target = 100
        for waypoint in way_points:
            dist = self.dist_euclid(waypoint[0],waypoint[1],targetX,targetY)
            if dist < dis_Near_target:
                dis_Near_target = dist
                NearX = waypoint[0]
                NearY = waypoint[1]
        
        self.goal_x,self.goal_y =self.map_car(NearX,NearY)
        self.mark_point(NearX,NearY,0,0,1.0,0.1,0.1) #rotationZ,rotationW)
        #self.set_cell() # occupancy grid
        
        return 

    def path_callback(self,pose_msg):
        print("==============",len(self.path))
        targetX = 1
        targetY = 0
        NearX  = targetX
        NearY = targetY
        dis_Near_target = 100
        for nd in self.path:
            dist = self.dist_euclid(nd.x,nd.y,targetX,targetY)
            if dist < dis_Near_target:
                dis_Near_target = dist
                NearX = nd.x
                NearY = nd.y
        mx,my = self.car_map(NearX,NearY)
        self.mark_point(mx,my,0,0,0.2,1.0,0.5) #rotationZ,rotationW)
        
        curve = 2*(NearY)/0.9**2
        angle = curve*0.4 #+3.14/4
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "pure" #"laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = VELOCITY

        if abs(angle) > 0.618:
            angle = angle/abs(angle)*0.618
        self.drive_pub.publish(drive_msg)
        #rospy.sleep(0.1)
        
        return 

    
    def find_path(self, latest_added_node):
        print("finding path")
        path = []
        nd = latest_added_node
        
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = 'points_and_lines'
        m.pose.orientation.w = 1.0
        m.action = Marker.ADD
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.color.a = 1.0
        m.scale.x = 0.03
        m.color.g = 0.8
        
        while(nd.is_root==False):
            path.insert(0,nd)
            nd = self.tree[nd.parent]
        
        for pt in path: 
            mx,my = self.car_map(pt.x,pt.y)
            p = Point(mx,my,0)
            m.points.append(p)
        self.iti_pub.publish(m)
        return path
    
        
        
# ==========================================================================================
    def draw_tree(self):
        M = MarkerArray()
        id = 0;
        for nd in self.tree:
            branch = []
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'points_and_lines'
            m.pose.orientation.w = 1.0
            m.action = Marker.ADD
            m.id = id
            id +=1
            m.type = Marker.LINE_STRIP
            m.color.a = 1.0
            m.scale.x = 0.03
            m.color.b = 0.5
            while(nd.is_root==False):
                branch.insert(0,nd)
                print("insert:",nd.x,nd.y,nd.parent)
                nd = self.tree[nd.parent]
            for pt in branch: 
                p = Point(pt.x,pt.y,0)
                m.points.append(p)
            M.markers.append(m)
        self.tree_pub.publish(M)
        print("finish draw tree")
        return
            
    def mark_point(self,aheadX,aheadY,aheadZ,aheadW,r,g,b):
            """
            Create slightly transparent disks for way-points.
            :param color: disk RGBA value
            """
            id = 0
            marker = Marker()
            marker.header.seq = 100
            marker.id = id
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE  # NOTE: color must be set here, not in rviz
            marker.action = Marker.ADD
            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = aheadX
            marker.pose.position.y = aheadY
            marker.pose.orientation.z = aheadZ
            marker.pose.orientation.w = aheadW
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 0.9
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            self.waypoint_pub.publish(marker);
            return


def main():
    with open("./waypoints_global.csv" ,'r') as f: 
        cr = csv.reader(f)
        for row in cr:
                array =[]
                for col in row:
                    array.append(eval(col))
                way_points.append(array)
    print(way_points[0])
    
    rospy.init_node('rrt')
    rospy.Rate(10)
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()