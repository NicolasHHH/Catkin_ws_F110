#!/usr/bin/python


import numpy as np
from numpy import linalg as LA
import math

from yaml import scan

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells

from tf2_ros import transform_listener
from tf.transformations import euler_from_quaternion


ARM_LENGHT = 1;

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self,x,y,parent_index,root=False):
        self.x = x
        self.y = y
        self.parent = parent_index
        self.cost = None # only used in RRT*
        
        self.is_root = root

# class def for RRT
class RRT(object):
    def __init__(self):
        # topics
        pf_topic = rospy.get_param('pose_topic')
        scan_topic = rospy.get_param('scan_topic')
        drv_topic = rospy.get_param('drive_topic')
        
        self.x = 0
        self.y = 0
        self.oz =0
        self.ow = 0
        self.vision =6.0
        self.dynamique = True
        self.tree = []

        # subscribers
        rospy.Subscriber(pf_topic, PoseStamped, self.pf_callback,queue_size=1)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback,queue_size=1)

        # publishers
        self.drive_pub = rospy.Publisher(drv_topic, AckermannDriveStamped, queue_size = 2)
        self.occ_pub = rospy.Publisher('map', OccupancyGrid, queue_size = 2,latch=False)
        self.map_pub = rospy.Publisher('map_metadata', MapMetaData, queue_size = 2,latch=False)
        self.cell_pub = rospy.Publisher('OccCell',GridCells,queue_size=1)
        self.red_pub = rospy.Publisher('TreeCell',GridCells,queue_size=2)
        
        #visualization==============================================================
        
        # Occupancy Grid
        self.resolution = rospy.get_param('Cell_size')
        self.grid_x = rospy.get_param('Grid_offset_x')
        self.grid_y = rospy.get_param('Grid_offset_y')
        self.width = rospy.get_param('Occupancy_width')
        self.height = rospy.get_param('Occupancy_height')
        
        self.occ_grid  = np.zeros((self.height,self.width))
        
        # Grid cell
        self.cells = GridCells()
        self.cells.header.frame_id="map"
        self.cells.cell_height = self.resolution
        self.cells.cell_width = self.resolution
        obstacle = Point()
        self.cells.cells.append(obstacle);
        self.cells.cells[0].x=0
        self.cells.cells[0].y=0
        self.cells.cells[0].z=0
        self.set_cell()
        self.samples = None
        self.samples = GridCells()
        self.samples.header.frame_id="map"
        self.samples.cell_height = self.resolution
        self.samples.cell_width = self.resolution
        
    def cood_grid(self,cx,cy):
        gx = int((cx-self.grid_x)/self.resolution)+self.height//2
        gy = int((cy-self.grid_y)/self.resolution)+self.width//2
        return [gx,gy]
    
    def grid_cood(self,gx,gy):
        cx = gx*self.resolution+self.grid_x-self.height/20
        cy = gy*self.resolution+self.grid_y-self.width/20
        return [cx,cy]
    
    # transfrom grid to cells ready to visualize the occupancy grid
    def set_cell(self):
        self.cells.cells.clear()
        for i in range(self.height):
            for j in range(self.width):
                if self.occ_grid[i][j]>0:
                    obstacle = Point()
                    obstacle.x,obstacle.y = self.grid_cood(i,j)
                    obstacle.z = 0
                    self.cells.cells.append(obstacle) 
    
    # single sample point to cell     
    def set_sample(self,x,y):
        obstacle = Point()
        obstacle.x = x
        obstacle.y = y
        obstacle.z = 0
        self.samples.cells.append(obstacle) 
        
    # called in sca_callback : fill a grid with 1
    def update_occupancy(self,rg,angle):
        
        euler = euler_from_quaternion((0,0,self.oz,self.ow))[2] 
        euler += angle
        cx = self.x+math.cos(euler)*rg
        cy = self.y+math.sin(euler)*rg
        gx,gy = self.cood_grid(cx,cy)
        if (0<gx and gx<self.height and gy>0 and gy<self.width):
            self.occ_grid[gx][gy] = 1
        return 
        
    def scan_callback(self, scan_msg):
        #print(self.ranges[0])
        length = len(scan_msg.ranges)
        step = 10
        
        if (self.dynamique == True):
            self.occ_grid = np.zeros((self.height,self.width))
        for i in range(0,length,step):
            rg = scan_msg.ranges[i]
            if rg > self.vision :  
                continue
            angle = scan_msg.angle_min+scan_msg.angle_increment*i
            self.update_occupancy(rg,angle)
        self.set_cell()
        #self.publish_map()

    def pf_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        """
        self.x = pose_msg.pose.position.x
        self.y = pose_msg.pose.position.y
        self.oz = pose_msg.pose.orientation.z
        self.ow = pose_msg.pose.orientation.w
        print(self.x,self.y)
        print(self.cood_grid(self.x,self.y))
        
        # root 
        self.tree.append(Node(self.x,self.y,0,True))
        
        ### test of uniform sampling
        """
        self.samples.cells.clear()
        for i in range(30):
            sx,sy = self.sample()
            self.set_sample(sx,sy)
        self.red_pub.publish(self.samples)
        """
        ### end of test
    
        self.cell_pub.publish(self.cells)
        return 

    def sample(self):
        xlim = 16
        ylim = 3
        intercept = 0.5
        """
        This method should randomly sample the free space, and returns a viable point
        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        rd = np.random.random()
        rd2 = np.random.random()
        if rd<0.25:
            ylim = -ylim
            intercept = 0
        elif (rd>=0.25 and rd<0.5):
            xlim = -xlim 
            intercept = 0.25
        elif (rd >0.75):
            xlim = -xlim 
            ylim = -ylim
            intercept = 0.75
            
        x = self.x+(rd-intercept)*xlim
        y = self.y+rd2*ylim*0.25
        if (self.cood_grid(x,y) == 1):
            x,y = self.sample()
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
        nearest_node = 0
        dis_min = 20
        print("Optimisation Goal: ",sampled_point)
        for i in range(len(self.tree)):
            dis = abs(self.tree[i].x-sampled_point[0])+abs(self.tree[i].y-sampled_point[1])
            if (dis_min > dis):
                dis_min = dis
                nearest_node = i
                print("iter: ",self.tree[i].x,self.tree[i].y)
        print("Optimised:", nearest_node);
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
        # ARM_LENGHT = 1;
        nx,ny = self.tree[nearest_node].x,self.tree[nearest_node].y
        sx,sy = sampled_point[0],sampled_point[1]
        dis = np.sqrt((nx-sx)**2+(ny-sy)**2)
        new_node = None
        if dis < ARM_LENGHT :
            sample = self.sample()
            print("steer again")
            return self.steer(self.nearest(sample),sample)
        else : 
            new_x, new_y = nx+(sx-nx)/dis,ny+(sy-ny)/dis
            if (self.cood_grid(new_x,new_y) == 0) :
                return None;
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
        
        amont_x,amont_y = self.cood_grid(nearest_node.x,nearest_node.y)
        aval_x,aval_y = self.cood_grid(new_node.x,new_node.y)
        width = 10 
        delta_x = amont_x - aval_x
        delta_y = amont_y - aval_y
        if abs(delta_y)>abs(delta_x):
            return False
        
        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        return path
    

    # ========================================================================================================

    def viz_message(self):
        
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin = Pose(Point(-self.grid_x, self.grid_y, 0),Quaternion(0, 0, 0, 1))
        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.occ_grid.reshape((self.occ_grid.size,)) * 50   #???
        grid_msg.data = [int(x) for x in flat_grid]
        
        for i in range(100): 
            grid_msg.data[i+200]=20
        #scan 
        return grid_msg
    
    def publish_map(self):
        grid_msg = self.viz_message()
        self.map_pub.publish(grid_msg.info)
        self.occ_pub.publish(grid_msg)
        return
    
    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straight line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhod of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood
    
###################################################""

def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()