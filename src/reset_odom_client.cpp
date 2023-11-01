#include <ros/ros.h>
#include "project1_22/ResetOdom.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_odom_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<project1_22::ResetOdom>("reset_odom");
    project1_22::ResetOdom srv;

    if (client.call(srv))
    {
        ROS_INFO("Resetted: [%d]", srv.response.resetted);
    }
    else
    {
        ROS_ERROR("Failed to call service reset_odom");
        return 1;
    }
}