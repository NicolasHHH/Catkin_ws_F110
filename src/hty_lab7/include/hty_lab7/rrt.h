
// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <queue>


// standard
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <random>
#include <cmath>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
class Node {
    public :
        double x, y,theta;
        int gr;
        int gc;
        double cost; 
        double hcost;
        //bool dead=false;
        int parent;
        int index; 
        //bool is_root = false;

        bool operator<(const Node& p2)const
        {
            return hcost > p2.hcost;
        }
} ;



class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need

    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    // ros::Subscriber drive_sub_;
    ros::Subscriber goal_sub_;

    ros::Publisher occ_pub;
    ros::Publisher drive_pub ;
    ros::Publisher vis_pub;

    // tf stuff
    tf::TransformListener listener;

    
    double car_X = 0;
    double car_Y = 0;
    double car_qz = 0;
    double car_qw = 0;
    double resolution = 0.5;
    double occupancy_width = 50;
    double occupancy_height = 50;
    double goal_X = 0;
    double goal_Y = 0;

    // params
    double angles[7] = {0,-0.2,0.2,-0.5,0.5,1,-1};
    double L  = 0.5;
    double penalty = 10;
    double SPEED = 3;
    int rayon = 3;

    double odom_rate ;
    double scan_rate ;
    double path_rate ;


    bool path_found = false;
    int moved = 0;
    std::vector<std::vector<double> > local_path; 
    double HEADER_DIS;

    //local map
    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid static_map;
    bool static_map_loaded = false;
    std::string occ_frame = "map";


    std::vector<Node> tree;
    int tree_count;



    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    

    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    // transforms 
    int rc_2_ind(int r, int c);
    std::vector<int> ind_2_rc(int ind) ;
    std::vector<int> coord_2_cell_rc(double x, double y);
    std::vector<double> rc_2_coord(int r, int c);

    int xy_2_ind(double x, double y);

    int static_xy_2_ind(double x, double y);
    int static_rc_2_ind(int r, int c);



    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node);
    std::vector<std::vector<double> > find_path( int last_added_index);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);
    void show_path();

};

