#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
static float max = 0;
static float min = 1;

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO_STREAM(msg->header);
  ROS_INFO("Intensities: [%f]",msg->intensities[0]);
  ROS_INFO("time:  %f",msg->scan_time);
  std::vector<float> recu = msg->ranges;
  for ( float i : recu){
     if (!std::isnan(i)  && !std::isinf(i)){
       if (i>max) max=i;
       if (i<min) min=i;
     }
  }
  ROS_INFO("max_range detected %f", max);
  ROS_INFO("min_range detected %f", min);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_from_lidar");
  
  ros::NodeHandle pn;
  ros::NodeHandle pn2;
  ros::NodeHandle nw;
  ros::Subscriber sub_scan = nw.subscribe("scan", 1000, chatterCallback);
  
  ros::Publisher chatter_pub  = pn.advertise<std_msgs::Float64>("farthest", 1000);
  ros::Publisher chatter_pub2 = pn2.advertise<std_msgs::Float64>("closest", 1000);
  ros::Rate loop_rate(10);
  std_msgs::Float64 far;
  std_msgs::Float64 near;
  far.data = 0.276;
  near.data = 3.145;

  while(ros::ok())
  {
    far.data = max;
    near.data = min;

    // no use
    ROS_INFO("pubs: far %f", far.data);

    chatter_pub.publish(far);
    chatter_pub2.publish(near);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}