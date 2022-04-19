
#include "hty_lab7/rrt.h"
#include <cmath>

using namespace std;

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {


    std::string pose_topic, scan_topic,drive_topic,map_topic,occ_topic,odom_topic,goal_topic;
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("drive_topic", drive_topic);
    nh_.getParam("map_topic", map_topic);
    nh_.getParam("occ_topic", occ_topic);
    nh_.getParam("odom_topic", odom_topic);
    nh_.getParam("goal_topic", goal_topic);

    
    nh_.getParam("resolution", resolution);
    nh_.getParam("Occupancy_width", occupancy_width);
    nh_.getParam("Occupancy_height", occupancy_height);
    nh_.getParam("occ_frame", occ_frame);

    nh_.getParam("header_dis", HEADER_DIS);
    nh_.getParam("speed", SPEED);
    nh_.getParam("penalty", penalty);
    nh_.getParam("rayon", rayon);

    nh_.getParam("odom_rate", odom_rate);
    nh_.getParam("scan_rate", scan_rate);
    nh_.getParam("path_rate", path_rate);

    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    map_sub_ = nh_.subscribe(map_topic, 10, &RRT::map_callback, this);
    odom_sub_ = nh_.subscribe(odom_topic, 10, &RRT::odom_callback, this);
    goal_sub_ = nh_.subscribe(goal_topic, 10, &RRT::goal_callback, this);
    
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    occ_pub = nh_.advertise<nav_msgs::OccupancyGrid>(occ_topic, 1);
    vis_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0 );

    map.header.frame_id=occ_frame;
    map.header.stamp = ros::Time::now(); 
    map.info.resolution = resolution;         // float32
    map.info.width      = occupancy_width;          // uint32
    map.info.height     = occupancy_height;          // uint32
    map.info.origin.position.x = car_X;
    map.info.origin.position.y = car_Y;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = car_qz;
    map.info.origin.orientation.w = car_qw;

}


// transformations
std::vector<int> RRT::ind_2_rc(int ind) {
    std::vector<int> rc;
    int row = floor(ind/map.info.width );
    int col = ind%map.info.width-1 ;
    rc.push_back(row);
    rc.push_back(col);
    return rc;
}

int RRT::rc_2_ind(int r, int c) {return r*map.info.width  + c;}

std::vector<int> RRT::coord_2_cell_rc(double x, double y) {
    std::vector<int> rc;
    rc.push_back(static_cast<int>((y-map.info.origin.position.y)/map.info.resolution));
    rc.push_back(static_cast<int>((x-map.info.origin.position.x)/map.info.resolution));
    return rc;
}
int RRT::xy_2_ind(double x, double y){
    return rc_2_ind(static_cast<int>((y-map.info.origin.position.y)/map.info.resolution),
        static_cast<int>((x-map.info.origin.position.x)/map.info.resolution));
}
std::vector<double> RRT::rc_2_coord(int r, int c){
    std::vector<double> xy;
    xy.push_back(c*map.info.resolution+map.info.origin.position.x);
    xy.push_back(r*map.info.resolution+map.info.origin.position.y);
    return xy;
}

// static map transformations
int RRT::static_rc_2_ind(int r, int c) {return r*static_map.info.width  + c;}
int RRT::static_xy_2_ind(double x, double y){
    return rc_2_ind(static_cast<int>((y-static_map.info.origin.position.y)/static_map.info.resolution),
        static_cast<int>((x-static_map.info.origin.position.x)/static_map.info.resolution));
}


// call_backs
void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    int length = 1080;
    int step = 1;
    int intercept = 50;
    float incre_angle = scan_msg->angle_increment;
    float min_angle = scan_msg->angle_min ;

    map.header.stamp = ros::Time::now(); 
    map.info.origin.position.x = car_X-occupancy_width*resolution/2;
    map.info.origin.position.y = car_Y-occupancy_width*resolution/2;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;//car_qz;
    map.info.origin.orientation.w = 0;//car_qw;
    int p[map.info.width*map.info.height] = {3}; 
    for(int i = intercept; i<length-intercept;i+=step){
        float rg = scan_msg->ranges[i];
        if (rg > occupancy_width/2*resolution){
            continue;
        }
        float angle = min_angle+incre_angle*i;
        tf::Matrix3x3 m(tf::Quaternion(0,0,car_qz,car_qw));
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        float euler = yaw;
        euler += angle;
        int ind = xy_2_ind(car_X+cos(euler)*rg, car_Y+sin(euler)*rg);

        for(int ii = -rayon;ii<=rayon; ii+=1){
            for(int jj = -rayon;jj<=rayon; jj+=1){
                int indd = ind+jj+ii*map.info.width;
                if(indd>0 && indd <map.info.width*map.info.height){
                    p[indd] = 95;
                }
            }
        }
    }
    std::vector<signed char> a(p, p+map.info.width*map.info.height);
    map.data = a;
}

void RRT::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    if (static_map_loaded){
        static_map.header = map_msg->header;
        static_map.info = map_msg->info;
        static_map.data = map_msg->data;
    }
}

void RRT::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg){
    path_found = false;
    goal_X = pose_msg->pose.position.x;
    goal_Y = pose_msg->pose.position.y;
    int r = coord_2_cell_rc(goal_X,goal_Y)[0];
    int c = coord_2_cell_rc(goal_X,goal_Y)[1];

    ROS_INFO("Goal received at %3f %3f, frame = map",goal_X,goal_Y);
    ROS_INFO("r,c, at %d %d, x,y: %3f %3f",r,c,
    c*map.info.resolution+map.info.origin.position.x,
    r*map.info.resolution+map.info.origin.position.y);

    /*
    for(int i = 0;i<map.info.width; i ++){
        for(int j = 0;j<map.info.width; j ++){
            int ind = rc_2_ind(i,j);
            ROS_INFO("r,c, at %d %d, x,y: %3f %3f",i,j,
                j*map.info.resolution+map.info.origin.position.x,
                i*map.info.resolution+map.info.origin.position.y);
                */



}

void RRT::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){

    car_X = odom_msg->pose.pose.position.x;
    car_Y = odom_msg->pose.pose.position.y;
    car_qw = odom_msg->pose.pose.orientation.w;
    car_qz = odom_msg->pose.pose.orientation.z;


    map.header.stamp = ros::Time::now(); 
    map.info.origin.position.x = car_X-occupancy_width*resolution/2;
    map.info.origin.position.y = car_Y-occupancy_width*resolution/2;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    map.info.origin.orientation.w = 0;
    ros::Rate rate(odom_rate);

    
    ROS_INFO("path length : %ld",local_path.size());
    show_path();

    if(!local_path.empty() && (car_X-goal_X)*(car_X-goal_X)+(car_Y-goal_Y)*(car_Y-goal_Y)>0.5){
        tf::Matrix3x3 m(tf::Quaternion(0,0,car_qz,car_qw));
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double targetX = car_X + cos(yaw)*HEADER_DIS;
        double targetY = car_Y + sin(yaw)*HEADER_DIS;
        double NearX  = targetX;
        double NearY = targetY;
        double dis_Near_target = 200;

        ROS_INFO("target : %f %f",NearX,NearY);
                    
        for(int i = 0; i<local_path.size();i++){
            double dist = sqrt((local_path[i][0]-targetX)*(local_path[i][0]-targetX)
            + (local_path[i][1]-targetY)*(local_path[i][1]-targetY));
            if (dist < dis_Near_target){
                dis_Near_target = dist;
                NearX = local_path[i][0];
                NearY = local_path[i][1];
            }
        }
        ROS_INFO("target : %f %f",NearX,NearY);

        double cy = (NearX-car_X)*sin(-yaw)+(NearY-car_Y)*cos(-yaw);

        ROS_INFO("cy : %f %f %f ",cy, car_X,car_Y);


        double angle = cy/HEADER_DIS/HEADER_DIS;
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "pure";
        drive_msg.drive.steering_angle = angle ; 
        if (abs(angle) > 0.718){
            angle = angle/abs(angle)*0.718;
        }
        drive_msg.drive.speed = SPEED-abs(angle);
        drive_pub.publish(drive_msg);
        moved += 1;
    }
    else{
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "pure";
        drive_msg.drive.steering_angle = 0 ; 
        drive_msg.drive.speed = 0;
        drive_pub.publish(drive_msg);
    }
    rate.sleep();

    occ_pub.publish(map);
}


void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    ros::Rate rate(path_rate);

    if (path_found && moved <100){
        ROS_INFO("PATH FOUND"); 
    }
    else{
        //return; 
        std::vector<Node> new_tree;
        tree= new_tree;
        tree_count = 0;

        int g_carR= coord_2_cell_rc(car_X,car_Y)[0];
        int g_carC= coord_2_cell_rc(car_X,car_Y)[1];

        priority_queue<Node> pq;
        Node root; 
        // root.is_root = true;

        tf::Matrix3x3 m(tf::Quaternion(0,0,car_qz,car_qw));
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        root.x, root.y,root.theta = car_X,car_Y,yaw ;
        root.gr = g_carR; // inverse
        root.gc = g_carC;
        root.cost=20; 
        double dist = (root.x-goal_X)*(root.x-goal_X)+(root.y-goal_Y)*(root.y-goal_Y);
        root.hcost = dist+root.cost;

        root.parent = tree_count;
        root.index = 0;
        tree.push_back(root);
        tree_count ++;
        pq.push(root);

        int iter = 400;

        int p[map.info.width*map.info.height] = {3};
        for(int i = 0;i<map.info.width*map.info.height;i++){
            p[i] = map.data[i];
        }
        p[rc_2_ind(root.gr,root.gc)] =12;
        
        
        while (!pq.empty() && iter>0){
            iter --; 
            Node nd = pq.top();
            pq.pop(); 

            if (is_goal(nd)){
                ROS_INFO("is_goal");
                //path_found = true;
                local_path = find_path(nd.index);
                return; 
            }
            
            for(int ii = -1;ii<=1; ii++ ) {
                for(int jj = -1;jj<=1; jj++ ){
                    if (ii+jj == 0 || ii+jj == 2 || ii+jj == - 2) continue; 
                    int c = nd.gc + jj;
                    int r = nd.gr + ii;
                    int id = rc_2_ind(r,c);
                    if(r<0  || c<0  || r >=map.info.width || c>=map.info.width) {
                        ROS_INFO("ind out of range with r:%d c:%d",r,c);
                        continue;
                    }
                    if(p[id]>10) continue; // < 10 free ; >10 filled or obstacle

                    Node new_nd; 
                    new_nd.y = r*map.info.resolution+map.info.origin.position.y;
                    new_nd.x = c*map.info.resolution+map.info.origin.position.x ; 

                    new_nd.gr = r;
                    new_nd.gc = c;
                    new_nd.cost = nd.cost +0.5;
                    new_nd.hcost = ((new_nd.x-goal_X)*(new_nd.x-goal_X)
                                    +(new_nd.y-goal_Y)*(new_nd.y-goal_Y))*penalty
                                    + new_nd.cost;
                    p[rc_2_ind(r,c)] = int(new_nd.hcost*0.01)+10;
                    ROS_INFO("Fill:(%d,%d),%d",r,c,int(new_nd.hcost*0.2));

                    new_nd.index = tree_count;
                    new_nd.parent = nd.index;
                    pq.push(new_nd);
                    tree.push_back(nd);
                    tree_count ++;

                    /*
                    std::vector<signed char> a(p, p+map.info.width*map.info.height);
                    map.data = a;
                    occ_pub.publish(map);
                    rate.sleep();
                    */
                    
                }
            }

        }
        std::vector<signed char> a(p, p+map.info.width*map.info.height);
        map.data = a;
        moved = 0;
        if(iter<1){
            ROS_INFO("Fail");
            return; 
        }
        //occ_pub.publish(map);
        rate.sleep();
    }
    
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    double x1 = nearest_node.x;
    double x2 = new_node.x;
    double y1 = nearest_node.y; 
    double y2 = new_node.y;
    int sample_n = 10;
    int bande_width = 1;
    double step_x = (x2-x1)/sample_n;
    double step_y = (y2-y1)/sample_n;
    for(int i = 1; i<sample_n;i++ ){

        x2 = x1+step_x*i;
        y2 = y1+step_y*i;

        int ind_current =  xy_2_ind(x2,y2);
        if(x2> car_X -occupancy_width/2*resolution && x2< car_X+occupancy_width/2*resolution 
        && y2> car_Y -occupancy_width/2 *resolution && y2< car_Y+occupancy_width/2*resolution){
            if (ind_current >=0 && ind_current < 10000 && map.data[ind_current]>90){
                return true;
            }
        }

        x2 = x1-bande_width*step_y+step_x*i;
        y2 = y1+bande_width*step_x+step_y*i;

        // x2> car_X -occupancy_width/2 && x2< car_X+occupancy_width/2 && y2> car_Y -occupancy_width/2 && y2< car_Y+occupancy_width/2

        ind_current =  xy_2_ind(x2,y2);
        if(x2> car_X -occupancy_width/2*resolution && x2< car_X+occupancy_width*resolution/2 
        && y2> car_Y -occupancy_width/2*resolution && y2< car_Y+occupancy_width*resolution/2){
            if (ind_current >=0 && ind_current < 10000 && map.data[ind_current]>80){
                return true;
            }
        }

        x2 = x1+bande_width*step_y+step_x*i;
        y2 = y1-bande_width*step_x+step_y*i;
        ind_current =  xy_2_ind(x2,y2);
        if(x2> car_X -occupancy_width/2*resolution && x2< car_X+occupancy_width/2*resolution 
        && y2> car_Y -occupancy_width/2*resolution && y2< car_Y+occupancy_width/2*resolution){
            if (ind_current >=0 && ind_current < 10000 && map.data[ind_current]>80){
                return true;
            }
        }
    }
    return false;
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    /*
    ros::Rate rate(5);
    for(int i = 0;i<40; i ++){
        double x = car_X + 3*cos(2*3.14/20 *i);
        double y = car_Y + 3*sin(2*3.14/20 *i);

        int p[map.info.width*map.info.height] = {3}; 
        int ind = xy_2_ind(x,y);
        int r = ind_2_rc(ind)[0];
        int c = ind_2_rc(ind)[1];
        ROS_INFO("r: %d, c:%d",r,c);
        x,y = r*map.info.resolution+map.info.origin.position.x,
        c*map.info.resolution+map.info.origin.position.y; 
        ROS_INFO("x: %f, y:%f",x,y);
        ind = xy_2_ind(x,y);
        p[ind] =50;
        std::vector<signed char> a(p, p+map.info.width*map.info.height);
        map.data = a;
        occ_pub.publish(map);
        rate.sleep();

    }
    for(int i = 0;i<map.info.width; i ++){
        for(int j = 0;j<map.info.width; j ++){
            int ind = rc_2_ind(i,j);
            int p[map.info.width*map.info.height] = {3}; 
            p[ind] =50;
            std::vector<signed char> a(p, p+map.info.width*map.info.height);
            map.data = a;
            occ_pub.publish(map);
            rate.sleep();
        }
    }
    for(int i = 0; i<map.info.width*map.info.height;i++){
        int p[map.info.width*map.info.height] = {3}; 
        for(int j = 0; j<i;j++){
            p[j] = 50;
        }
        std::vector<signed char> a(p, p+map.info.width*map.info.height);
        map.data = a;
        occ_pub.publish(map);
        rate.sleep();
    }
    */
    
    return sampled_point;
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    // TODO: fill in this method

    return new_node;
}

bool RRT::is_goal(Node & node) {

    if ((node.x-goal_X)*(node.x-goal_X)+(node.y-goal_Y)*(node.y-goal_Y)<0.3){
        return true;
    }
    return false;
}

std::vector<std::vector<double> > RRT::find_path(int last_added_index) {

    std::vector<std::vector<double> > found_path;
    int current = last_added_index;
    int i = 0; 
    int PAS = 1;
    ROS_INFO("find_path : ");

    while(tree[current].index!=0){
        if (i==PAS){
            i = 0;
            std::vector<double> point; 
            point.push_back(tree[current].x);
            point.push_back(tree[current].y);
            found_path.push_back(point);
            ROS_INFO("find current at :%f %f;",tree[current].x,tree[current].y);
        }
        i++;
        current = tree[current].parent;

    }
    ROS_INFO("root reached !");
    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}

void RRT::show_path(){
    visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "points_and_lines";
        m.pose.orientation.w = 1.0;
        m.action = visualization_msgs::Marker::ADD;
        m.id = 1;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.color.a = 1.0;
        m.scale.x = 0.03;
        m.color.g = 0.8;
        int i = 0; 
        int PAS = 1;
        for (int j = 0;j<local_path.size();j++){
            i++;
            if (i==PAS){
                geometry_msgs::Point p;
                p.x = local_path[j][0];
                p.y = local_path[j][1];
                p.z = 0;
                m.points.push_back(p);
                i = 0;
            } 
        }
        vis_pub.publish(m);

}
        
