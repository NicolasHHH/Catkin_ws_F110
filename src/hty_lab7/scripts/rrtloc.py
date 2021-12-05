#!/usr/bin/python

import numpy as np
import math

from yaml import scan
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
from nav_msgs.msg import GridCells
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from tf2_ros import transform_listener
from tf.transformations import euler_from_quaternion
import tf


ARM_LENGHT = 0.6

# pure pursuit
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters
VELOCITY = 0.5 # meters per second
HEADER_DIS = 2*CAR_LENGTH 
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
        # topics
        pf_topic = rospy.get_param('pose_topic')
        scan_topic = rospy.get_param('scan_topic')
        drv_topic = rospy.get_param('drive_topic')
        self.resolution = rospy.get_param('Cell_size')
        
        self.x = 0
        self.y = 0
        self.oz =0
        self.ow = 0
        self.vision =6.0
        self.tree = []
        self.goal_x =0
        self.goal_y =0
        
        # pure pursuit
        self.waypoints = way_points
        self.carA = 0
        self.path = []
        self.tf_listener = tf.TransformListener()
        
        # Grid cell
        self.cells = GridCells()
        self.cells.header.frame_id="map"
        self.cells.cell_height = self.resolution
        self.cells.cell_width = self.resolution
        
        self.samples = None
        self.samples = GridCells()
        self.samples.header.frame_id="map"
        self.samples.cell_height = self.resolution
        self.samples.cell_width = self.resolution
        
        
        # subscribers
        rospy.Subscriber(pf_topic, PoseStamped, self.pf_callback,queue_size=1)
        #rospy.Subscriber(pf_topic, PoseStamped, self.path_callback,queue_size=1)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback,queue_size=1)

        # publishers
        self.drive_pub = rospy.Publisher(drv_topic, AckermannDriveStamped, queue_size = 2)
        self.cell_pub = rospy.Publisher('OccCell',GridCells,queue_size=1)
        self.red_pub = rospy.Publisher('TreeCell',GridCells,queue_size=2)
        self.iti_pub = rospy.Publisher("itin_marker", Marker, queue_size=20)
        self.tree_pub = rospy.Publisher("tree_marker", MarkerArray, queue_size=20)
        self.waypoint_pub = rospy.Publisher('waypoint_vis',Marker,queue_size = 2)
        
        # Occupancy Grid
        self.award  = 50
        self.expand = 60
        self.occ_grid = np.ones((self.expand+20,self.award+20))
        self.set_cell()
        self.occ_grid  = np.zeros((self.expand+20,self.award+20))
        
        
    # cood in car frame
    def cood_grid(self,cx,cy):
        # slef.x-29 <= cx <= slef.x+29
        gx = self.expand//2+10 +int((cx-self.x)/self.resolution)
        # slef.y <= cy <= slef.y +50
        gy = int((cy-self.y)/self.resolution)+10
        print("cood_grid",cx,cy,gx,gy)
        return [gx,gy]
    # cood in car frame
    def grid_cood(self,gx,gy):
        cx = (gx-self.expand//2-10)*self.resolution+self.x
        cy = (gy-10)*self.resolution+self.y
        return [cx,cy]
    
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
    
    # transfrom grid to cells ready to visualize the occupancy grid
    def set_cell(self):
        self.cells.cells.clear()
        for i in range(self.expand):
            for j in range(self.award):
                if self.occ_grid[i][j]==1:
                    cx,cy = self.grid_cood(i,j)
                    obstacle = Point()
                    obstacle.x,obstacle.y = self.car_map(cx,cy)
                    obstacle.z = 0
                    self.cells.cells.append(obstacle) 
        self.cell_pub.publish(self.cells)
    
    # single sample point to cell     
    def set_sample(self,x,y):
        obstacle = Point()
        obstacle.x,obstacle.y,obstacle.z = x,y,0
        self.samples.cells.append(obstacle) 
        
    # called in sca_callback : fill a grid with 1
    def update_occupancy(self,rg,angle):
        cx = self.x+math.cos(angle)*rg
        cy = self.y+math.sin(angle)*rg
        gx,gy = self.cood_grid(cx,cy)
        if (0<=gx and gx<50 and gy>=0 and gy<60):
            self.occ_grid[gx][gy] = 1
        return 
        
    def scan_callback(self, scan_msg):
        self.occ_grid = np.ones((self.expand+20,self.award+20))
        self.set_cell()
        
        length = len(scan_msg.ranges)
        step = 8
        intercept = 150
        incre_angle = scan_msg.angle_increment
        min_angle = scan_msg.angle_min 
        
        self.occ_grid = np.zeros((self.expand+20,self.award+20))
            
        for i in range(intercept,length-intercept,step):
            rg = scan_msg.ranges[i]
            if rg > self.vision :  
                continue
            angle = min_angle+incre_angle*i
            self.update_occupancy(rg,angle)
            
        # root 
        self.tree = []
        self.tree.append(Node(0,0,-1,True)) # in car frame
        latest_node = self.tree[0]
        i=0
        while self.is_goal(latest_node) == False:
            i+=1
            sample_point = self.sample()
            nearest_node_index = self.nearest(sample_point)
            new_node = self.steer(nearest_node_index,sample_point)
            if self.check_collision(self.tree[nearest_node_index],new_node):
                latest_node = new_node
                self.tree.append(new_node)
            print("iteration: ",i)
        #self.draw_tree()
        self.path = self.find_path(latest_node) 
        
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
                
        #self.mark_point(NearX,NearY,0,0) #rotationZ,rotationW)
        self.goal_x =NearX
        self.goal_y =NearY
        
        #self.red_pub.publish(self.samples)
        #self.samples.cells.clear()
        self.set_cell() # occupancy grid
        #self.cell_pub.publish(self.cells)
        return 

    def path_callback(self,pose_msg):
        print("==============",len(self.path))
        targetX = self.x + math.cos(self.carA)
        targetY = self.y + math.sin(self.carA)
        NearX  = targetX
        NearY = targetY
        dis_Near_target = 100
        for nd in self.path:
            dist = self.dist_euclid(nd.x,nd.y,targetX,targetY)
            if dist < dis_Near_target:
                dis_Near_target = dist
                NearX = nd.x
                NearY = nd.y
        print(self.x,self.y,NearX,NearY)
                
        self.mark_point(NearX,NearY,0,0) #rotationZ,rotationW)

        aheadPoint = PointStamped()
        aheadPoint.header.frame_id = "map"
        aheadPoint.point.x = NearX
        aheadPoint.point.y = NearY
        aheadPoint.point.z = 0
        
        transformed_point = pose_msg
        transformed_point = self.tf_listener.transformPoint("base_link",aheadPoint)
        
        curve = 2*(transformed_point.point.y)/0.9**2
        angle = curve*0.4 #+3.14/4
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "pure" #"laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = VELOCITY

        if abs(angle) > 0.418:
            angle = angle/abs(angle)*0.418

        #self.drive_pub.publish(drive_msg)
        
        return 

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point
        Args:
        Returns:
            (x, y) (float float): a tuple in car frame

        """
        rdx = np.random.random()
        rdy = np.random.random()
        gx = int(rdx*self.award)
        gy = int(rdy*self.expand)
        
        while(self.occ_grid[gx][gy]==1):
            rdx = np.random.random()
            rdy = np.random.random()
            gx = int(rdx*self.award)
            gy = int(rdy*self.expand)
        x,y = self.grid_cood(gx,gy)
        print("sample: ",x,y,"grid",gx,gy)
        return (x, y)

    def nearest(self, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        #print("nearest")
        nearest_node = 0
        dis_min = 20
        #print("Optimisation Goal: ",sampled_point)
        for i in range(len(self.tree)):
            dis = self.dist_euclid(self.tree[i].x,self.tree[i].y,sampled_point[0],sampled_point[1])
            if (dis_min > dis):
                dis_min = dis
                nearest_node = i
                #print("iter: ",self.tree[i].x,self.tree[i].y)
        print("Nearest found:", nearest_node)
        
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        print("steering")
        # ARM_LENGHT = 1;
        nx,ny = self.tree[nearest_node].x,self.tree[nearest_node].y
        sx,sy = sampled_point[0],sampled_point[1]
        dis =  self.dist_euclid(nx,ny,sx,sy)
        new_node = None
        if dis < ARM_LENGHT :
            return Node(sx,sy,nearest_node)
        else : 
            new_x, new_y = nx+(sx-nx)/dis*ARM_LENGHT,ny+(sy-ny)/dis*ARM_LENGHT
            new_node = Node(new_x, new_y,nearest_node)
            return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        print("check collision")
        amont_x,amont_y = self.cood_grid(nearest_node.x,nearest_node.y)
        aval_x,aval_y = self.cood_grid(new_node.x,new_node.y)
        
        current_x,current_y = nearest_node.x,nearest_node.y
        delta_y = abs(amont_y - aval_y)
        if delta_y == 0: return True
        y_unit = (nearest_node.y-new_node.y)/delta_y
        x_unit = (nearest_node.x-new_node.x)/delta_y
        
        for i in range(delta_y):
            gx,gy = self.cood_grid(current_x,current_y)
            for j in range(5):
                if self.occ_grid[gx+j][gy]== 1 or self.occ_grid[gx-j][gy]== 1:
                    new_node.parent = -1
                    return False
            current_x+=x_unit
            current_y+=y_unit

        return True

    def is_goal(self, latest_added_node):
        print("check goal")
        return (abs(latest_added_node.x-self.goal_x)+abs(latest_added_node.y-self.goal_y)<ARM_LENGHT)

    def find_path(self, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
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
            
    def mark_point(self,aheadX,aheadY,aheadZ,aheadW):
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
            marker.color.r = 1.0
            self.waypoint_pub.publish(marker);
            return


def main():
    with open("waypoint6_100.csv" ,'r') as f: 
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