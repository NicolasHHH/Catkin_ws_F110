#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;


  //parent frame
  transformStamped.header.frame_id = "turtle1";
  transformStamped.child_frame_id = "carrot1";
  double rot =0;
  //ROS_INFO("%d",ros::Time::now().toSec());
  transformStamped.transform.translation.x = 2.0;
  transformStamped.transform.translation.y = 2.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
        q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
 
  ros::Rate rate(10.0);
  while (node.ok()){
    rot += 0.05;
    if (rot > 6.28) rot = 0;
    transformStamped.transform.translation.x = 2.0*sin(rot);
    transformStamped.transform.translation.y = 2.0*cos(rot);
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }
}