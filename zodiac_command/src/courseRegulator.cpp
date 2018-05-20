/*------------------------------------------------------------------------------
* ROS node "courseRegulator" subscribes to "fix", "imu" and "desired_course" topics. 
* It publishes "helm_cmd" topic.
*
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Int32.h"


using namespace std;

ros::Publisher helmCmd_pub;

// The helm command corespond to the angle of the hord board motor with the boat (in degree)
std_msgs::Int32 helmCmd_msg;

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

}

void desiredCourse_callback(const std_msgs::Int32::ConstPtr& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "courseRegulator");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ros::Subscriber fix_sub = nh.subscribe("fix", 1, fix_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu", 1, imu_callback);
    ros::Subscriber desiredCourse_sub = nh.subscribe("desired_course", 1, desiredCourse_callback);
    
    helmCmd_pub = nh.advertise<std_msgs::Int32>("helm_angle_cmd", 1);

    double loopRate;
    nhp.param<double>("courseRegulator/loop_rate", loopRate, 1);
    ros::Rate loop_rate(loopRate);

    while (ros::ok())
    {
        helmCmd_pub.publish(helmCmd_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}