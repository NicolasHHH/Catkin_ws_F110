#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Bool.h>
#include <vector>


class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    // TODO: create ROS subscribers and publishers
    ros::Subscriber sub_scan;
    ros::Subscriber sub_odom;
    ros::Publisher pub_break;
    ros::Publisher pub_bool;
    

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        pub_break = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake", 2);
        pub_bool= n.advertise<std_msgs::Bool>("brake_bool", 2);
        sub_scan = n.subscribe("scan", 1000, &Safety::scan_callback,this);
        sub_odom = n.subscribe("odom", 1000, &Safety::odom_callback,this);


    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        // TODO: update current speed
        //speed = 0.0;
        speed = odom_msg->twist.twist.linear.x ;

    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        double range = scan_msg->range_min;
        double angle = scan_msg->angle_min;
        double angle_incre = scan_msg->angle_increment;
        bool TTC = false  ;  // 
        double ttc_seuil = 0.9;
        //ros::Rate loop_rate(2);
        std::vector<float> dis= scan_msg->ranges;

        if (speed > 0.1 || speed < -0.1) {
                angle = scan_msg->angle_min;
                for (size_t i = 0; i < dis.size(); i++) {
                    // TTC calculations
                    // calculate projected velocity
                    double proj_velocity = speed * std::cos(angle);
                    angle += angle_incre*2;
                    i++;
                    double ttc = dis[i] / proj_velocity;
                    // if it's small enough to count as a collision
                    //- car_distances[i]
                    //ROS_INFO("dis: %f  speed: %f  ",dis[i],speed);
                    //ROS_INFO("proj: %f angle: %f  TTC: %f ",proj_velocity,angle,ttc);
                    if (ttc < ttc_seuil && (ttc >= 0.0)) { 
                        TTC = true ;
                        ROS_INFO("AEB dis:%f angle:%f speed:%f ttc:%f",dis[i],angle,speed,ttc);
                        std_msgs::Bool  bool_msg;
                        bool_msg.data = TTC;
                        pub_bool.publish(bool_msg);
                        ackermann_msgs::AckermannDriveStamped ack_msg;
                        ack_msg.drive.speed = 0;
                        pub_break.publish(ack_msg);
                    }
                }
            }
        //loop_rate.sleep();
        // TODO: publish drive/brake message

    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "Safety_node");
    Safety sn;  // safety node;
    //ros::Rate loop_rate(5);
    ros::spin();
    return 0;
}