/*------------------------------------------------------------------------------
* ROS node "lineFollowing" subscribes to "fix" and "waypoint_line" topics. 
* It publishes "desired_course" topic.
* It computes the desierd,course of the boat in order to follow
* lines given by two waypoints.
*
* Developer Notes:
*    Algorithm inspired and modified from:
*    - Luc Jaulin and Fabrice Le Bars "An Experimental Validation of a Robust Controller with
*       the VAIMOS Autonomous Sailboat" [1];
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <zodiac_command/WaypointListMission.h>
#include <zodiac_command/mathUtility.h>
#include <sensor_msgs/NavSatFix.h>
#include "std_msgs/Int32.h"

using namespace std;

#define DATA_OUT_OF_RANGE -2000

ros::Publisher desiredCourse_pub;
std_msgs::Int32 desiredCourse_msg;

vector<zodiac_command::WaypointMission> waypointLine;
double boatLatitude = DATA_OUT_OF_RANGE;
double boatLongitude = DATA_OUT_OF_RANGE;

double incidenceAngle;   // degrees
double maxDistanceFromLine; // meters
double waypointRadius; // meters


double calculateAngleOfDesiredTrajectory(const double m_nextWaypointLon, const double m_nextWaypointLat, 
    const double m_prevWaypointLon, const double m_prevWaypointLat, const double m_VesselLon, const double m_VesselLat)
{
    const int earthRadius = 6371000; //meters

    array<double, 3> prevWPCoord = {
        earthRadius * cos(mathUtility::degreeToRadian(m_prevWaypointLat)) * cos(mathUtility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * cos(mathUtility::degreeToRadian(m_prevWaypointLat)) * sin(mathUtility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * sin(mathUtility::degreeToRadian(m_prevWaypointLat))};

    array<double, 3> nextWPCoord = {
        earthRadius * cos(mathUtility::degreeToRadian(m_nextWaypointLat)) * cos(mathUtility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * cos(mathUtility::degreeToRadian(m_nextWaypointLat)) * sin(mathUtility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * sin(mathUtility::degreeToRadian(m_nextWaypointLat))};

    double M[2][3] = {
        {-sin(mathUtility::degreeToRadian(m_VesselLon)), cos(mathUtility::degreeToRadian(m_VesselLon )), 0},
        {-cos(mathUtility::degreeToRadian(m_VesselLon ))*sin(mathUtility::degreeToRadian(m_VesselLat )),
         -sin(mathUtility::degreeToRadian(m_VesselLon ))*sin(mathUtility::degreeToRadian(m_VesselLat )),
          cos(mathUtility::degreeToRadian(m_VesselLat ))}
        };

    array<double, 3> bMinusA = { nextWPCoord[0]-prevWPCoord[0], nextWPCoord[1]-prevWPCoord[1], nextWPCoord[2]-prevWPCoord[2]};

    // 2x3 * 3x1
    double phi = atan2(M[0][0]*bMinusA[0] + M[0][1]*bMinusA[1] + M[0][2]*bMinusA[2],
        M[1][0]*bMinusA[0] + M[1][1]*bMinusA[1] + M[1][2]*bMinusA[2]);

    return phi;  // in north east down reference frame.
}

double calculateTargetCourse(const double m_nextWaypointLon, const double m_nextWaypointLat, 
    const double m_prevWaypointLon, const double m_prevWaypointLat, const double m_VesselLon, const double m_VesselLat)
{
    // Calculate signed distance to the line.           [1] and [2]: (e).
    double signedDistance = mathUtility::calculateSignedDistanceToLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
        m_prevWaypointLat, m_VesselLon, m_VesselLat);
    // std::cout << "signedDistance : " << signedDistance <<std::endl;

    // Calculate the angle of the line to be followed.  [1]:(phi)       [2]:(beta)
    double phi = calculateAngleOfDesiredTrajectory(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
        m_prevWaypointLat, m_VesselLon, m_VesselLat);
    // std::cout << "phi : " << phi <<std::endl;

    // Calculate the target course in nominal mode.     [1]:(theta_*)   [2]:(theta_r)
    double targetCourse = phi + (2 * mathUtility::degreeToRadian(incidenceAngle)/M_PI) * atan(signedDistance/maxDistanceFromLine);
    targetCourse = mathUtility::limitRadianAngleRange(targetCourse); // in north east down reference frame.
    // std::cout << "targetCourse: " << targetCourse <<std::endl;

    targetCourse = mathUtility::radianToDegree(targetCourse);

    return targetCourse; // in north east down reference frame.

}

void ifBoatPassedOrEnteredWP_setPrevWPToBoatPos(const double m_nextWaypointLon, const double m_nextWaypointLat, 
    const double m_prevWaypointLon, const double m_prevWaypointLat, const double m_VesselLon, const double m_VesselLat)
{
    double distanceAfterWaypoint = mathUtility::calculateWaypointsOrthogonalLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
            m_prevWaypointLat, m_VesselLon, m_VesselLat);

    double DTW = mathUtility::calculateDTW(m_VesselLon, m_VesselLat, m_nextWaypointLon, m_nextWaypointLat);

    if(distanceAfterWaypoint > 0 ||  DTW < waypointRadius)
    {
        waypointLine.at(0).longitude = boatLongitude;
        waypointLine.at(0).latitude = boatLatitude;
    }
}


void waypointLine_callback(const zodiac_command::WaypointListMission::ConstPtr& msg)
{
    ROS_INFO("New waypoint line recived");
    waypointLine = msg->waypoints;
}

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg)
{
    if (fix_msg->status.status >= fix_msg->status.STATUS_FIX)
    {
        boatLatitude = fix_msg->latitude;
        boatLongitude = fix_msg->longitude;
    }
    else
    {
        ROS_WARN("No gps fix");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lineFollowing");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ros::Subscriber waypointLine_sub = nh.subscribe("waypoint_line", 1, waypointLine_callback);
    ros::Subscriber fix_sub = nh.subscribe("fix", 1, fix_callback);
    
    desiredCourse_pub = nh.advertise<std_msgs::Int32>("desired_course", 1);

    nhp.param<double>("lineFollowing/incidence_angle", incidenceAngle, 90);
    nhp.param<double>("lineFollowing/max_distance_from_line", maxDistanceFromLine, 20);
    nhp.param<double>("waypointMgr/waypoint_radius", waypointRadius, 5);
    // cout << "incidenceAngle=" << incidenceAngle << endl;

    double loopRate;
    nhp.param<double>("lineFollowing/loop_rate", loopRate, 1);
    ros::Rate loop_rate(loopRate);

    while (ros::ok())
    {
        if((boatLatitude != DATA_OUT_OF_RANGE) && (boatLongitude != DATA_OUT_OF_RANGE) && (waypointLine.size()>0))
        {
            ifBoatPassedOrEnteredWP_setPrevWPToBoatPos(waypointLine.at(1).longitude, waypointLine.at(1).latitude, 
            waypointLine.at(0).longitude, waypointLine.at(0).latitude, boatLongitude, boatLatitude);
            
            double targetCourse = calculateTargetCourse(waypointLine.at(1).longitude, waypointLine.at(1).latitude, 
            waypointLine.at(0).longitude, waypointLine.at(0).latitude, boatLongitude, boatLatitude);

            desiredCourse_msg.data = (int) targetCourse;
            desiredCourse_pub.publish(desiredCourse_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}