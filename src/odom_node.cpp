#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <sstream>
#include <ackermann_odom_tf/Odom.h>
#include "ackermann_odom_tf/ResetOdom.h"

#define L 2.8

class OdomNode {
    private:
        ros::NodeHandle n;

        ros::Publisher pub_odom;
        ros::Publisher pub_custom;
        ros::Subscriber sub;
        tf::TransformBroadcaster br;

        float x, y, th;
        float speed, steering_angle;

        ros::Time prev_time;
        
        ros::ServiceServer service;

    public:

        OdomNode() {
            pub_odom = n.advertise<nav_msgs::Odometry>("odometry", 10);
            pub_custom = n.advertise<ackermann_odom_tf::Odom>("custom_odometry", 10);
            sub = n.subscribe("/speed_steer", 1,  &OdomNode::callback, this);

            // Retrieve parameters for x, y, z
            if(!n.getParam("/starting_x", x) 
            || !n.getParam("/starting_y", y) 
            || !n.getParam("/starting_th", th)) {
                ROS_INFO("Impossible to retrieve parameters.");
            } else {
                ROS_INFO("Parameters retrieved, node initialized. X: %f, Y: %f, THETA: %f", x, y, th);
            }

            prev_time = ros::Time(0.0);

            service = n.advertiseService("reset_odom", &OdomNode::reset, this);
        }

        void publishTF() {
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(x, y, 0) );
            tf::Quaternion q;
            q.setRPY(0, 0, th);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
        }

        void publishOdom() {
            nav_msgs::Odometry odom;
            ackermann_odom_tf::Odom custom_odom;
            custom_odom.x = x;
            custom_odom.y = y;
            custom_odom.th = th;

            std_msgs::Header header;
            header.stamp = ros::Time::now();
            std::stringstream ss;
            ss << header.stamp;
            custom_odom.timestamp = ss.str();

            pub_custom.publish(custom_odom);
        }

        void computeOdom() {
            // compute odometry
            
            ROS_INFO("SPEED: %f, STEERING_ANGLE: %f", speed, steering_angle );

            if(prev_time.toSec() == 0.0) {
                prev_time = ros::Time::now();
            }

            ros::Time current_time = ros::Time::now();

            float dt = (current_time - prev_time).toSec();
            prev_time = current_time;

            float d_x = 0;
            float d_y = 0;
            float d_th = 0;

            if (steering_angle != 0.0) {

                float R = L / tan(steering_angle);
                float omega = speed / R;
                d_th = omega * dt;

            } 

            th += d_th;

            d_x = speed * cos(th) * dt;
            d_y = speed * sin(th) * dt;

            ROS_INFO("d_X: %f, d_Y: %f, d_THETA: %f, TIME: %f", d_x, d_y, d_th, dt);
            x += d_x;
            y += d_y;

            ROS_INFO("X: %f, Y: %f, THETA: %f\n-----------------------------------", x, y, th);
            publishOdom();
            publishTF();
        }

        void callback(const geometry_msgs::Quaternion::ConstPtr& q) {
            speed = q->x;
            steering_angle = q->y;

            computeOdom();
        }

        bool reset(ackermann_odom_tf::ResetOdom::Request &req, ackermann_odom_tf::ResetOdom::Response &res) {
            x = 0;
            y = 0;
            th = 0;
            ROS_INFO("Sending back response: [%d]", res.resetted);
            return true;
        }

        ros::NodeHandle get_handle() {
            return n;
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_node");
    OdomNode on;

    ros::spin();
    return 0;
}