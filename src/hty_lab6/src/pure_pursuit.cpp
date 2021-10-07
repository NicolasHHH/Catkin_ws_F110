#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries you need
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>

class PurePursuit {
private:
    ros::NodeHandle n;
    // TODO: create ROS subscribers and publishers
    ros::Subscriber sub_scan;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_odom;
    ros::Publisher pub_drive;
    double speed;
    double L = 2; // final 

public:
    PurePursuit() {
        n = ros::NodeHandle();
        sub_scan = n.subscribe("scan", 1000, &PurePursuit::scan_callback,this);
        sub_pose = n.subscribe("pose", 1000, &PurePursuit::pose_callback,this);
        sub_odom = n.subscribe("odom", 1000, &PurePursuit::odom_callback,this);
        pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 2);
        // TODO: create ROS subscribers and publishers
    }
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        ros::Rate rate(2);

    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        speed = odom_msg->twist.twist.linear.x ;
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
        // TODO: find the current waypoint to track using methods mentioned in lecture

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle
        double angle = 0;
        double py = pose_msg->pose.position.y;
        double px = pose_msg->pose.position.x;
        double orient = pose_msg->pose.orientation.x/(pose_msg->pose.orientation.y+pose_msg->pose.orientation.x);
        

        // TODO: publish drive message, 
        // don't forget to limit the steering angle between -0.4189 and 0.4189 radians

        if (angle >0.4189){
            angle = 0.4189;
        }
        if (angle <-0.4189){
            angle = -0.4189;
        }

        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = speed;
        pub_drive.publish(drive_msg);
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}