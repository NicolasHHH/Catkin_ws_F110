#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Time.h>
// TODO: include ROS msg type headers and libraries
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <vector>


class Gap_follow{

    private:
        ros::NodeHandle n;
        ros::Subscriber sub_scan;
        ros::Publisher pub_drive;
        const int window = 10;  
        // un petit window est plus sensible aux petits obstactles 

        const int plafond = 3.6; //4
        // plafond grand: moins de turbulence
        // plafond petit: sensible aux petits obstactles( moins moyenné par les grandes distances)

        const int rayon = 100; //90
        // rayon of bubble;

        const double seuil = 0.4;
        // rue étroite

    public:
        Gap_follow(){
            n = ros::NodeHandle();
            // publisher and subscriber
            sub_scan = n.subscribe("scan", 600 , &Gap_follow::scan_callback,this); //1000
            pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 2); // 2

        }
        std::vector<float> preprocess_lidar(std::vector<float>& ranges){
            /*Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
            */
            std::vector<float> proc_ranges = ranges;
            int len = proc_ranges.size();
            for (int i = 0; i<len-window; i++ ){
                double sum_local = 0.0;
                for (int j = i;j<i+window;j++){
                    if (proc_ranges[j] > plafond or std::isinf(proc_ranges[j]) or std::isnan(proc_ranges[j])){
                        proc_ranges[j] = plafond;
                    }
                    sum_local += proc_ranges[j];
                }
                proc_ranges[i] = sum_local / window;
            }

            return proc_ranges;
        }

        std::vector<int> find_max_gap(std::vector<float> ranges){
            // Return the start index & end index of the max gap in free_space_ranges
           std::vector<int> intervalle(2);
           int start=0,end=0;
           int max_len =0;
           int flag =0;
           for(int i= 0; i<ranges.size(); i++){
               if (ranges[i] > seuil && flag == 0){
                   start = i;
                   flag = 1;
               }
               if (flag ==1 && (ranges[i] < seuil or i>ranges.size()-3)){
                   end = i-1;
                   flag = 0;
               }
               if (end-start>max_len){
                   max_len = end - start; 
                   intervalle[0]=start;
                   intervalle[1]=end;
               }
           }
           return intervalle;
        }
    
        int find_best_point(std::vector<int> intervalle, std::vector<float> ranges){
            /*Start_i & end_i are start and end indicies of max-gap range, respectively
                Return index of best point in ranges
                Naive: Choose the furthest point within ranges and go there
            */
            const int start = intervalle[0];
            const int end = intervalle[1];
            int middle = (start+end)/2;
            int max_range = 0;
            int pos = middle; 
            for (int i = 0;i < middle-start;i++ ){
                if (ranges[i+middle]- i*0.05/(end-start)> max_range){ //0.99 0.25 
                    pos = i+middle; 
                    max_range = ranges[pos];
                }
                if (ranges[middle-i]- i*0.05/(end-start)> max_range){
                    pos = middle-i;
                    max_range = ranges[pos];
                }
            }
            //ROS_INFO("start: %d end %d pos: %d",start,end,pos);
            return pos;
        }

        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
            /*Process each LiDAR scan as per the Follow Gap algorithm & 
              publish an AckermannDriveStamped Message
            */
            //ros::Rate rate(1000); // sleep
            std::vector<float> ranges = scan_msg->ranges;
            std::vector<float> proc_ranges = preprocess_lidar(ranges);

            //#Find closest point to LiDAR
            double near = 100;
            int near_pos = 0;
            int len = proc_ranges.size();
            for (int i = 0; i<len/2;i++){
                if ( proc_ranges[i]<near){
                    near_pos = i;
                    near = proc_ranges[i];
                }
                if ( proc_ranges[len-1-i]<near){
                    near_pos = len-1-i;
                    near = proc_ranges[len-1-i];
                }
            }
            //#Eliminate all points inside 'bubble' (set them to zero) 
            for (int i = std::max(0,near_pos-rayon); i<std::min(len-1,near_pos+rayon);i++){
                proc_ranges[i] = 0;
            }

            //#Find max length gap 
            std::vector<int> max_gap = find_max_gap(proc_ranges);

            //#Find the best point in the gap 
            int best_point = find_best_point(max_gap,proc_ranges);

            //#Publish Drive message
            double angle = scan_msg->angle_min + best_point*scan_msg->angle_increment;
            double velocity ;
            //ROS_INFO("best point: %u max_gap: %d",best_point,max_gap[1]-max_gap[0]);
            //ROS_INFO("angle: %f range: %f",angle,proc_ranges[best_point]);
            if (std::abs(angle) < 10.0/180*3.14){
                velocity = 1.8;
            }
            else if (std::abs(angle) < 20.0/180*3.14){
                velocity = 1.5;
            }
            else if (std::abs(angle) < 30.0/180*3.14){
                velocity = 1.2;
            }
            else{ velocity = 0.9;}
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.header.frame_id = "laser";
            drive_msg.drive.steering_angle = angle;
            drive_msg.drive.speed = velocity;
            pub_drive.publish(drive_msg);
            //rate.sleep();
        }


};

int main(int argc, char ** argv) {

    ros::init(argc, argv, "Gap_follow_node");
    Gap_follow gf;
    ros::spin();
    return 0;
}