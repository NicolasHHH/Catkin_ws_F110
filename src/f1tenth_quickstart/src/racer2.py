#!/usr/bin/env python
import numpy as np
import math
#from yaml import scan
import csv
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import tf

# pure pursuit
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters
#VELOCITY = 1.5 # meters per second
#HEADER_DIS = 2.5*CAR_LENGTH 
global way_points 
way_points= []
DURATION = 20 # ten times laser scan


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
class Agent(object):
    def __init__(self):
        # topics
        pf_topic = "/opp_id/odom"
        scan_topic = "/opp_id/scan"
        self.resolution = 0.2
        
        self.x,self.y = 0,0
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
        self.waypoints = way_points
        self.carA = 0
        self.path = []
        self.gx,self.gy = self.cood_grid(0,0)
        self.queue = [Node(self.gx,self.gy,None,True)]
        #self.tf_listener = tf.TransformListener()
        
        
        # subscribers
        rospy.Subscriber(pf_topic, Odometry, self.pf_callback,queue_size=1)
        rospy.Subscriber(pf_topic, Odometry, self.path_callback,queue_size=1)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback,queue_size=1)

        # publishers
        self.drive_pub = rospy.Publisher("/opp_id/drive", AckermannDriveStamped, queue_size = 2)
        self.iti_pub = rospy.Publisher("/itin_marker", Marker, queue_size=2)
        self.waypoint_pub = rospy.Publisher('/waypoint_vis',Marker,queue_size = 2)
        
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
        
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        self.oz = pose_msg.pose.pose.orientation.z
        self.ow = pose_msg.pose.pose.orientation.w
        euler = euler_from_quaternion([0,0,self.oz,self.ow])
        self.carA  = euler[2]
        #print("pose:",self.x,self.y,self.carA)
        
        targetX = self.x + math.cos(self.carA)*self.header_dis
        targetY = self.y + math.sin(self.carA)*self.header_dis
        NearX  = targetX
        NearY = targetY
        dis_Near_target = 20
        for waypoint in way_points:
            dist = self.dist_euclid(waypoint[0],waypoint[1],targetX,targetY)
            if dist < dis_Near_target:
                dis_Near_target = dist
                NearX = waypoint[0]
                NearY = waypoint[1]
        
        self.goal_x,self.goal_y =self.map_car(NearX,NearY)
        #self.mark_point(NearX,NearY,0,0,1.0,0.1,0.1) #rotationZ,rotationW)
        
        return 

    def path_callback(self,pose_msg):
        #print("==============",len(self.path))
        targetX = 1 #self.header_dis
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
        #self.mark_point(mx,my,0,0,0.2,1.0,0.5) #rotationZ,rotationW)
        
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

    def show_path(self):
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


def main():
    with open("/home/tianyang/f110_ws/waypoint6_100.csv" ,'r') as f: 
        cr = csv.reader(f)
        for row in cr:
                array =[]
                for col in row:
                    array.append(eval(col))
                way_points.append(array)
    print(way_points[0])
    
    rospy.init_node('opp_agent')
    opp_agent = Agent()
    rospy.spin()

if __name__ == '__main__':
    main()