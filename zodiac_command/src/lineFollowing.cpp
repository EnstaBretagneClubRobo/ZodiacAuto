/*------------------------------------------------------------------------------
* ROS node "lineFollowing" subscribes to "fix" and "waypoint_line" topics. 
* It publishes "desired_course" topic.
*
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <zodiac_command/WaypointListMission.h>
#include <sensor_msgs/NavSatFix.h>
#include "std_msgs/Int32.h"


using namespace std;

ros::Publisher desiredCourse_pub;

std_msgs::Int32 desiredCourse_msg;

void waypointLine_callback(const zodiac_command::WaypointListMission::ConstPtr& msg)
{

}

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lineFollowing");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ros::Subscriber waypointLine_sub = nh.subscribe("waypoint_line", 1, waypointLine_callback);
    ros::Subscriber fix_sub = nh.subscribe("fix", 1, fix_callback);
    
    desiredCourse_pub = nh.advertise<std_msgs::Int32>("desired_course", 1);

    // float kp;
    // nhp.param<float>("lineFollowing/incidence_angle", kp, 0);
    // cout << "kp=" << kp << endl;

    double loopRate;
    nhp.param<double>("lineFollowing/loop_rate", loopRate, 1);
    ros::Rate loop_rate(loopRate);

    while (ros::ok())
    {
        desiredCourse_pub.publish(desiredCourse_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}