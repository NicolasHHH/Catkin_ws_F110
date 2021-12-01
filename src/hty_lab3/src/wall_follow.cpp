#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Time.h>
// TODO: include ROS msg type headers and libraries
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <vector>

//double VELOCITY = 3.50; // meters per second par défaut

class Wall_follow{
    private:
        // PID params
        const double Kp = 1;
        const double Kd = 0.1;
        const double Ki = 0.01;

        double servo_offset = 0.0;
        double prev_error = 0.0 ;
        double error = 0.0;
        double integral = 0.0;
        double VELOCITY = 3.50;
        int bl = ros::param::get("/vitesse", VELOCITY);
    	
        
        // WALL FOLLOW PARAMS
        const int ANGLE_RANGE = 270; // Hokuyo 10LX has 270 degrees scan
        const double DESIRED_DISTANCE_RIGHT = 2.1; // meters
        const double DESIRED_DISTANCE_LEFT = 2.0;
        const double CAR_LENGTH = 0.50; // Traxxas Rally is 20 inches or 0.5 meters

        // ROS 
        ros::NodeHandle n;
        ros::Subscriber sub_scan;
        ros::Subscriber sub_odom;
        ros::Publisher pub_drive;
        double speed;

    public:
        Wall_follow(){
            n = ros::NodeHandle();
            // publishers and subscribers
            sub_scan = n.subscribe("scan", 1000, &Wall_follow::scan_callback,this);
            sub_odom = n.subscribe("odom", 1000, &Wall_follow::odom_callback,this);
            pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 2);

        }
        double middle(const sensor_msgs::LaserScan::ConstPtr &scan_msg){

            return 0;
        }
        double getRange(double dis_b, double alpha){
            // data: single message from topic /scan
            // angle: between -45 to 225 degrees, where 0 degrees is directly to the right
            // Outputs length in meters to object with angle in lidar scan field of view
            //make sure to take care of nans etc.
            //TODO: implement 
            // if (!std::isnan()  && !std::isinf(i)){}
            return dis_b * std::cos(alpha) + speed*sin(alpha);
        }
        void pid_control(double error, double velocity){

            //TODO: Use kp, ki & kd to implement a PID controller for 
            integral = prev_error + error;
            double angle = Kp * error + Kd * (error - prev_error) + Ki * integral;
            prev_error = error;
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.header.frame_id = "laser";
            drive_msg.drive.steering_angle = angle;
            drive_msg.drive.speed = velocity;
            pub_drive.publish(drive_msg);

        }
        double followLeft(double leftDist){ 
            // calculate error
            double error =  DESIRED_DISTANCE_LEFT - leftDist;
            return error;
        }
        void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
            speed = odom_msg->twist.twist.linear.x ;
        }

        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
            ros::Rate rate(2);
            double angle = scan_msg->angle_min;
            ROS_INFO("%f",angle);
            double angle_incre = scan_msg->angle_increment;
            std::vector<float> dis= scan_msg->ranges;
            double dis_a, dis_b, angle_a, angle_b;
            int flag = 0;
            for (size_t i = 0; i < dis.size(); i+=10) {
                if (angle > -1.57 && angle < -1.4 && flag == 0){
                    dis_b = dis[i];
                    flag = 1;
                    angle_b = angle;
                }
                if (angle > -1.0 && angle < -0.8 && flag == 1){
                    dis_a = dis[i];
                    flag = 2;
                    angle_a = angle;
                }
                angle += angle_incre*10;
            }

            //ROS_INFO("angle_a: %f angle_b: %f dis_a: %f dis_b: %f", angle_a,angle_b,dis_a,dis_b);
            double alpha = std::atan( (dis_a*std::cos(angle_a-angle_b)-dis_b)/(dis_a*std::sin(angle_a-angle_b)));
            double leftDist = getRange(dis_b,alpha);
            double error = followLeft(leftDist);// TODO: replace with error returned by followLeft

            ROS_INFO("alpha = %f leftDist = %f error = %f", alpha, leftDist,error);
            ROS_INFO("VELOCITY = %f",VELOCITY);
            // send error to pid_control
            double velocity = VELOCITY;
            if (abs(alpha *180/3.14) < 10){
                velocity = 0.75*VELOCITY;
            }
            else if (abs(alpha *180/3.14) < 20){
                velocity = 0.6*VELOCITY;
            }
            else{
                velocity = 0.45*VELOCITY;
            }

            pid_control(error, velocity);
        }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "Wall_follow_node");
    Wall_follow wf;
    ros::spin();
    
    return 0;
}
