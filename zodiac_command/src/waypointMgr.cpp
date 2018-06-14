/*------------------------------------------------------------------------------
* ROS node "waypointMgr" subscribes to "fix" and "new_waypoint_mission" topics. 
* It publishes "status_waypoint_mission" and "waypoint_line" topics.
* It is a state machine that give the line to follow at the line following node.
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <zodiac_command/WaypointListMission.h>
#include <zodiac_command/mathUtility.h>

using namespace std;

ros::Publisher waypointLine_pub;
ros::Publisher statusWaypointMission_pub;

vector<zodiac_command::WaypointMission> waypointMission;
double boatLatitude;    // degrees
double boatLongitude;   // degrees
bool missionRunning = 0;

double waypointRadius;  // meters


bool isNextWaypointReached()
{
    for (int i=0; i<waypointMission.size(); i++)
    {
        if (waypointMission.at(i).waypointReached == 0)
        {
            //Calculate distance to waypoint
            double DistanceToWaypoint = mathUtility::calculateDTW(boatLongitude, boatLatitude, waypointMission.at(i).longitude, waypointMission.at(i).latitude);
            // cout << "DistanceToWaypoint: " << DistanceToWaypoint << endl;
            // cout << "Waypoint Radius: " << waypointRadius << endl;
            if(DistanceToWaypoint > waypointRadius)
            {
                return false;
            }
            else
            {
                ROS_INFO("Waypoint reached");
                waypointMission.at(i).waypointReached = 1;
                return true;
            }
        }
    }


    return true;
}

void publishWaypointLine()
{
    zodiac_command::WaypointListMission waypointLine_msg;

    waypointLine_msg.header.stamp = ros::Time::now();
    waypointLine_msg.header.frame_id = "waypoint_line";
    waypointLine_msg.child_frame_id = "map";

    for (int i=0; i<waypointMission.size(); i++)
    {
        if (waypointMission.at(i).waypointReached == 0)
        {
            if(i==0){
                zodiac_command::WaypointMission boatPose;
                boatPose.waypointID = 0;
                boatPose.latitude  = boatLatitude;
                boatPose.longitude = boatLongitude;
                boatPose.waypointReached = 1;
                waypointLine_msg.waypoints.push_back(boatPose);
            }
            else{
                waypointLine_msg.waypoints.push_back(waypointMission.at(i-1));
            }
            waypointLine_msg.waypoints.push_back(waypointMission.at(i));
            break;
        }
    }

    waypointLine_pub.publish(waypointLine_msg);
}

void publishStatusWaypointMission()
{
    zodiac_command::WaypointListMission statusWaypointMission_msg;

    statusWaypointMission_msg.header.stamp = ros::Time::now();
    statusWaypointMission_msg.header.frame_id = "status_waypoint_mission";
    statusWaypointMission_msg.child_frame_id = "map";
    statusWaypointMission_msg.waypoints = waypointMission;

    statusWaypointMission_pub.publish(statusWaypointMission_msg);
}


void newWaypointMission_callback(const zodiac_command::WaypointListMission::ConstPtr& msg)
{
    ROS_INFO("New waypoint mission received");
    missionRunning = 1;    
    waypointMission = msg->waypoints;
    publishStatusWaypointMission();

}

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg)
{
    if (fix_msg->status.status >= fix_msg->status.STATUS_FIX)
    {
        boatLatitude = fix_msg->latitude;
        boatLongitude = fix_msg->longitude;
        if(missionRunning)
        {
            if (isNextWaypointReached())
            {
                publishStatusWaypointMission();
            }
            publishWaypointLine();
        }
    }
    else
    {
        ROS_WARN_THROTTLE(5, "No gps fix");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypointMgr");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");


    ros::Subscriber newWaypointMission_sub = nh.subscribe("new_waypoint_mission", 1, newWaypointMission_callback);
    ros::Subscriber fix_sub = nh.subscribe("fix", 1, fix_callback);
    
    waypointLine_pub = nh.advertise<zodiac_command::WaypointListMission>("waypoint_line", 1);
    statusWaypointMission_pub = nh.advertise<zodiac_command::WaypointListMission>("status_waypoint_mission", 1000, true);

    nhp.param<double>("waypointMgr/waypoint_radius", waypointRadius, 5);

    ros::spin();


    return 0;
}
