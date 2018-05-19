/*------------------------------------------------------------------------------
* ROS node "waypointMgr" subscribes to "fix" and "new_waypoint_mission" topics. 
* It publishes "status_waypoint_mission" and "waypoint_line" topics.
* It is a state machine that give the line to follow at the line following node.
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <zodiac_command/WaypointListMission.h>
#include <sensor_msgs/NavSatFix.h>


using namespace std;

ros::Publisher waypointLine_pub;
ros::Publisher statusWaypointMission_pub;

zodiac_command::WaypointListMission waypointLine_msg;
zodiac_command::WaypointListMission statusWaypointMission_msg;

void newWaypointMission_callback(const zodiac_command::WaypointListMission::ConstPtr& msg)
{
    // zodiac_command::WaypointMission WP0 = msg->waypoints[0];
    // zodiac_command::WaypointMission WP1 = msg->waypoints[1];
    // cout << "lat=" << WP0.latitude << endl;
    // cout << "lon=" << WP0.longitude << endl;
    // cout << "lat=" << WP1.latitude << endl;
    // cout << "lon=" << WP1.longitude << endl;
}

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypointMgr");
    ros::NodeHandle nh;

    ros::Subscriber newWaypointMission_sub = nh.subscribe("new_waypoint_mission", 1, newWaypointMission_callback);
    ros::Subscriber fix_sub = nh.subscribe("fix", 1, fix_callback);
    
    waypointLine_pub = nh.advertise<zodiac_command::WaypointListMission>("waypoint_line", 1);
    statusWaypointMission_pub = nh.advertise<zodiac_command::WaypointListMission>("status_waypoint_mission", 1000);

    ros::spin();


    return 0;
}
