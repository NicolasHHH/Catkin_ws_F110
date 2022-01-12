#!/usr/bin/python
import numpy as np
import math

from yaml import scan
import csv

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import GridCells
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion
import tf

# pure pursuit
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters
#VELOCITY = 1.5 # meters per second
#HEADER_DIS = 2.5*CAR_LENGTH 
path_point = []
DURATION = 20 # ten times laser scan

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self,x,y,parent,root):
        self.gx,self.gy = x,y
        self.parent = parent
        self.is_root = root
        return
        
    def loc(self):
        return (self.gx,self.gy)
    
    def back_to_root(self):
        itin = []
        node = self
        pas = 0
        while (node.is_root == False):
            pas += 1
            itin.append(node.loc())
            node = node.parent
        print("Path de ",pas," pas.")
        return itin

# class def for RRT
class BFS(object):
    def __init__(self):
        # topics
        pf_topic = "/gt_pose"
        scan_topic = "/scan"
        self.resolution = 0.2
        
        self.x,self.y = 0,0
        self.ox,self.oy = 0,0
        self.oz,self.ow = 0,0
        self.vision =6.0
        self.goal_x =0
        self.goal_y =0
        self.dfs_time = 0 # modulo DURATION 
        self.speed = 1.5
        self.dfs_ok = False
        
        # Occupancy Grid
        self.award, self.expand = 50,50
        self.extra = 0
        self.occ_grid  = np.zeros((self.award+self.extra,self.expand+self.extra))
        
        # pure pursuit
        self.LOOKAHEAD_DISTANCE = 1.70#1.70 # meters
        self.VELOCITY = 3 # m/s
        #self.goal = 0
        self.goal_idx = 0
        self.velocity = 2.5#1.5
        self.tf_listener = tf.TransformListener()
        
        rospy.Subscriber("/gt_pose", PoseStamped, self.callback, queue_size=1)
        self.read_waypoints()
        
        # Grid cell
        self.cells = GridCells()
        self.cells.header.frame_id="map"
        self.cells.cell_height = self.resolution
        self.cells.cell_width = self.resolution
        
        self.samples = GridCells()
        self.samples.header.frame_id="map"
        self.samples.cell_height = self.resolution
        self.samples.cell_width = self.resolution
        
        # subscribers
        rospy.Subscriber(pf_topic, PoseStamped, self.pf_callback,queue_size=1)
        rospy.Subscriber(pf_topic, PoseStamped, self.path_callback,queue_size=1)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback,queue_size=1)

        # publishers
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 2)
        self.cell_pub = rospy.Publisher('OccCell',GridCells,queue_size=1)
        #self.red_pub = rospy.Publisher('TreeCell',GridCells,queue_size=2)
        self.iti_pub = rospy.Publisher("itin_marker", Marker, queue_size=2)
        self.waypoint_pub = rospy.Publisher('waypoint_vis',Marker,queue_size = 2)
        
    # cood in car frame
    def cood_grid(self,cx,cy):
        gx = int(cx/self.resolution)
        gy = int(cy/self.resolution)+self.expand//2+self.extra//2
        return [gx,gy]
    # cood in car frame
    def grid_cood(self,gx,gy):
        cx = (gx)*self.resolution
        cy = (gy-self.expand//2-self.extra//2)*self.resolution
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
        
    def dfs(self):
        self.queue = [Node(self.gx,self.gy,None,True)]
        
        ggx,ggy = self.cood_grid(self.goal_x,self.goal_y)
        pas = 1
        iter = 0
        self.dfs_ok = False
        while len(self.queue)>0:
            iter += 1
            v = self.queue.pop(0)
            if  abs(v.gx-ggx)<=pas and abs(v.gy-ggy)<=pas:
                self.path = v.back_to_root()
                self.dfs_ok = True
                break
            for i,j in [(1,0),(1,1),(1,-1),(0,1),(0,-1)]: #(1,0),(0,-1),(0,1),(-1,0),
                i = i*pas
                j = j*pas
                if 0<= v.gx+i <=self.award and 0<= v.gy+j <=self.expand:
                    clr = self.occ_grid[v.gx+i][v.gy+j]
                    if clr!=1 and clr != 0.2:
                        self.queue.append(Node(v.gx+i,v.gy+j,v,False))
                        self.occ_grid[v.gx+i][v.gy+j] = 0.2
        print("iter:",iter)   
        return 
    
    def set_cell(self):
        self.cells.cells = []
        for i in range(self.award+self.extra):
            for j in range(self.expand+self.extra):
                if self.occ_grid[i][j]==1:
                    cx,cy = self.grid_cood(i,j)
                    obstacle = Point()
                    obstacle.x,obstacle.y = self.car_map(cx,cy)
                    obstacle.z = 0
                    self.cells.cells.append(obstacle) 
        self.cell_pub.publish(self.cells)
        
    # called in sca_callback : fill a grid with 1
    def update_occupancy(self,rg,angle):
        cx = math.cos(angle)*rg
        cy = math.sin(angle)*rg
        gx,gy = self.cood_grid(cx,cy)
        if (self.extra//2<=gx and gx<self.award+self.extra//2 and gy>=self.extra//2  and gy<self.expand+self.extra//2 ):
            #self.occ_grid[gx][gy] = 1
            self.occ_grid[gx-1][gy-1] = 1
            self.occ_grid[gx-1][gy+1] = 1
            self.occ_grid[gx+1][gy-1] = 1
            self.occ_grid[gx+1][gy+1] = 1
            self.occ_grid[gx+1][gy] = 1
            self.occ_grid[gx][gy+1] = 1
            self.occ_grid[gx][gy-1] = 1
            self.occ_grid[gx-1][gy] = 1
        return 
        
    def scan_callback(self, scan_msg):
        self.cells.cells= [] # refresh 
        self.occ_grid = np.zeros((self.award+self.extra,self.expand+self.extra))

        length = len(scan_msg.ranges)
        step = 6
        intercept = 150
        incre_angle = scan_msg.angle_increment
        min_angle = scan_msg.angle_min 
        for i in range(intercept,length-intercept,step):
            rg = scan_msg.ranges[i]
            if rg > self.vision :  
                continue
            angle = min_angle+incre_angle*i
            self.update_occupancy(rg,angle)
        
        if (self.dfs_time <= DURATION//2):
            for yy in range(-1,2):
                for xx in range(1,10):
                    if self.occ_grid[xx][yy+self.expand//2]==1:
                        self.dfs_time = DURATION
                        break
                
        ggx,ggy = self.cood_grid(self.goal_x,self.goal_y)
        self.speed = 3
        if ggx <=3+self.extra//2 or abs(ggy-self.expand//2-self.extra//2)>8:
            self.dfs_time = 0
            self.speed = 1.3 
            
        print("dfs_time:", self.dfs_time)
        if (self.dfs_time >0):
            print("dfs------")
            self.speed = 1.5
            self.header_dis = 2.5*CAR_LENGTH 
            self.dfs()
            if(self.dfs_ok==False):
                self.path =[self.cood_grid(self.goal_x,self.goal_y)]
            self.dfs_time -=1
        else :
            print("waypoint follow------")
            self.header_dis = 1*CAR_LENGTH 
            self.path =[self.cood_grid(self.goal_x,self.goal_y)]
        print("header_dis", self.header_dis)
        #print(self.path)
        self.show_path()
        #rospy.sleep(0.1)
        return
        
    def pf_callback(self, pose_msg):
        
        self.x = pose_msg.pose.position.x
        self.y = pose_msg.pose.position.y
        self.ox = pose_msg.pose.orientation.x
        self.oy = pose_msg.pose.orientation.y
        self.oz = pose_msg.pose.orientation.z
        self.ow = pose_msg.pose.orientation.w
        euler = euler_from_quaternion([0,0,self.oz,self.ow])
        yaw = self.carA  = euler[2]
        
        self.path_points_x = np.array(self.path_points_x)
        self.path_points_y = np.array(self.path_points_y)

        ## finding the distance of each way point from the current position 
        for i in range(len(self.path_points_x)):
            self.dist_arr[i] = self.dist((self.path_points_x[i],self.path_points_y[i]),(self.x,self.y))

        ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
        goal_arr = np.where((self.dist_arr < self.LOOKAHEAD_DISTANCE+0.3)&(self.dist_arr > self.LOOKAHEAD_DISTANCE-0.3))[0]
        # goal_arr : goal indexes
        
        # find those are ahead
        for idx in goal_arr:
            #line from the point position to the car position
            v1 = [self.path_points_x[idx]-self.x , self.path_points_y[idx]-self.y]
            #since the euler was specified in the order x,y,z the angle is wrt to x axis
            v2 = [np.cos(yaw), np.sin(yaw)]
            #find the angle between these two vectors NOTE:These are in world coordinates
            temp_angle = self.find_angle(v1,v2)
            if abs(temp_angle) < np.pi/2:
                self.goal_idx = idx
                break

        ##finding the distance of the goal point from the vehicle coordinator
        L = self.dist_arr[self.goal_idx]
        
        aheadPoint = PointStamped()
        aheadPoint.header.frame_id = "map"
        aheadPoint.point.x = self.path_points_x[self.goal_idx]
        aheadPoint.point.y = self.path_points_y[self.goal_idx]
        aheadPoint.point.z = 0
        # TODO: transform goal point to vehicle frame of refercence
        transformed_point = pose_msg
        transformed_point = self.tf_listener.transformPoint("base_link",aheadPoint)

        self.goal_x,self.goal_y =transformed_point.point.x,transformed_point.point.y
        self.mark_point(self.goal_x,self.goal_y,0,0,1.0,0.1,0.1) #rotationZ,rotationW)
        #self.set_cell() # occupancy grid
        
        return 

    def path_callback(self,pose_msg):
        #print("==============",len(self.path))
        targetX = 0.5 #self.header_dis
        targetY = 0
        NearX  = targetX
        NearY = targetY
        dis_Near_target = 10
        for nd in self.path:
            ndx,ndy = self.grid_cood(nd[0],nd[1])
            dist = self.dist_euclid(ndx,ndy,targetX,targetY)
            if dist < dis_Near_target:
                dis_Near_target = dist
                NearX = ndx
                NearY = ndy
        mx,my = self.car_map(NearX,NearY)
        self.mark_point(mx,my,0,0,0.2,1.0,0.5) #rotationZ,rotationW)
        
        curve = 2*(NearY)/0.9**2
        angle = curve*0.4 #+3.14/4
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "pure" #"laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.speed-abs(angle)

        if abs(angle) > 0.718:
            angle = angle/abs(angle)*0.<718
            drive_msg.drive.speed = self.speed-abs(angle)
        self.drive_pub.publish(drive_msg)
        #rospy.sleep(0.1)
        
        return 

    def read_waypoints(self):
        #rospack = rospkg.RosPack()
        #package_path=rospack.get_path('a_stars_pure_pursuit')
        #filename=package_path+'/waypoints/waypoints_1.csv'
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_w   = [float(point[2]) for point in path_points]
        #Initialize array of zeros with the same number of entries as the waypoint markers
        self.dist_arr= np.zeros(len(self.path_points_y))

    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # USE THIS FUNCTION IF CHANGEABLE SPEED IS NEEDED
    def set_speed(self,angle):
        if (abs(angle)>0.2018):
            self.LOOKAHEAD_DISTANCE = 1.2
            # self.velocity = 1.5
            self.angle = angle

            if self.velocity - 1.5 >= 0.5:
                self.velocity -= 0.5#0.7

        else:
            self.LOOKAHEAD_DISTANCE = 1.2
            # self.velocity = 3.0
            self.angle = angle

            if self.VELOCITY - self.velocity > 0.2:
                self.velocity += 0.2
        print(angle,self.velocity)

    # USE THIS FUNCTION IF CONSTANT SPEED IS NEEDED
    def const_speed(self,angle):
        #self.LOOKAHEAD_DISTANCE = 2
        self.angle = angle
        self.velocity = 1.5#self.VELOCITY

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)

    def show_path(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = 'points_and_lines'
        m.pose.orientation.w = 1.0
        m.action = Marker.ADD
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.color.a = 0.7
        m.scale.x = 0.03
        m.color.r = 0.9
        m.color.g = 0
        i,pas = 0,3
        for pt in self.path: 
            i+=1
            if (i==pas):
                cx,cy = self.grid_cood(pt[0],pt[1])
                mx,my = self.car_map(cx,cy)
                p = Point(mx,my,0)
                m.points.append(p)
                i = 0
        self.iti_pub.publish(m)
        return
    
        
        
# ==========================================================================================
            
    def mark_point(self,aheadX,aheadY,aheadZ,aheadW,r,g,b):
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



if __name__ == '__main__':
    with open("waypoint6_100.csv" ,'r') as f: 
        cr = csv.reader(f)
        path_points = [tuple(line) for line in csv.reader(f)]
    print(path_points[0])

    rospy.init_node('BFS')
    bfs = BFS()
    rospy.spin()