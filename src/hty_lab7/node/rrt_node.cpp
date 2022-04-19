

#include "hty_lab7/rrt.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt");
    ros::NodeHandle nh;
    RRT rrt(nh);
    ros::Rate rate(0.5);
    ros::spin();
    return 0;
}
