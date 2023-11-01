#include <ros/ros.h>
#include "project1_22/ResetOdom.h"

bool reset(project1_22::ResetOdom::Request &req, project1_22::ResetOdom::Response &res) {
    ROS_INFO("Sending back response: [%d]", res.resetted);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_odom_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("reset_odom", reset);

    ROS_INFO("Ready to reset odometry.");
    ros::spin();

    return 0;
}