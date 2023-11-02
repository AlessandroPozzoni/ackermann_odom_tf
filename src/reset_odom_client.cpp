#include <ros/ros.h>
#include "ackermann_odom_tf/ResetOdom.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_odom_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ackermann_odom_tf::ResetOdom>("reset_odom");
    ackermann_odom_tf::ResetOdom srv;

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