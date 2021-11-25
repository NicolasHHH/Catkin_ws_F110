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


# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        
        self.is_root = False # ??

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
        self.vision =4.0

        # subscribers
        rospy.Subscriber(pf_topic, PoseStamped, self.pf_callback,queue_size=1)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback,queue_size=1)

        # publishers
        self.drive_pub = rospy.Publisher(drv_topic, AckermannDriveStamped, queue_size = 2)
        self.occ_pub = rospy.Publisher('map', OccupancyGrid, queue_size = 2,latch=False)
        self.map_pub = rospy.Publisher('map_metadata', MapMetaData, queue_size = 2,latch=False)
        self.cell_pub = rospy.Publisher('BlueCell',GridCells,queue_size=1)
        self.yellow_pub = rospy.Publisher('YellowCell',GridCells,queue_size=2)
        
        #visualization
        
        # Occupancy Grid
        self.resolution = rospy.get_param('Cell_size')*2
        self.grid_x = rospy.get_param('Grid_offset_x')
        self.grid_y = rospy.get_param('Grid_offset_y')
        self.width = rospy.get_param('Occupancy_width')//2
        self.height = rospy.get_param('Occupancy_height')//2
        
        self.occ_grid  = np.zeros((self.width,self.height))
        
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
        
    def cood_grid(self,cx,cy):
        gx = int( (cy-self.grid_x)/self.resolution+self.width)
        gy = int((cx+self.grid_y)/self.resolution+self.height)
        return [gx,gy]
    
    def grid_cood(self,gx,gy):
        cy = (gx-self.width)*self.resolution+self.grid_x
        cx = (gy-self.height)*self.resolution-self.grid_y
        return [cx,cy]
        
    def set_cell(self):
        self.cells.cells.clear()
        for i in range(self.width):
            for j in range(self.height):
                if self.occ_grid[i][j]>0:
                    obstacle = Point()
                    obstacle.x,obstacle.y = self.grid_cood(i,j)
                    obstacle.z = 0
                    self.cells.cells.append(obstacle) 
    
    def update_occupancy(self,rg,angle):
        euler = euler_from_quaternion((0,0,self.oz,self.ow))[2] 
        euler += angle
        cx = self.x+math.cos(euler)*rg
        cy = self.y+math.sin(euler)*rg
        gx,gy = self.cood_grid(cx,cy)
        if (0<gx and gx<300 and gy>0 and gy<220):
            self.occ_grid[gx][gy] = 1
        return 
        
    def scan_callback(self, scan_msg):
        #print(self.ranges[0])
        length = len(scan_msg.ranges)
        step=20
        #for i in range(self.width):
         #   for j in range(self.height):
          #      self.occ_grid[i][j] = 0
        for i in range(0,length,step):
            rg = scan_msg.ranges[i]
            if rg > self.vision :  
                continue
            angle = scan_msg.angle_min+scan_msg.angle_increment*i
            #print(angle,rg)
            self.update_occupancy(rg,angle)
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
        self.set_cell()
        self.cell_pub.publish(self.cells)
        #print(self.cood_grid(self.x,self.y))
        return 

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = None
        y = None
        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
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
        new_node = None
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

def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()