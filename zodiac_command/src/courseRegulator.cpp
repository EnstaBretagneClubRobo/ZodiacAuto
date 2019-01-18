/*------------------------------------------------------------------------------
* ROS node "courseRegulator" subscribes to "vel", "imu" and "desired_course" topics.
* It publishes "helm_cmd" topic.
* It also suscribes to "waypointLine" for the implementation of regulatorSlalom
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float64.h"
#include <zodiac_command/WaypointListMission.h>
#include <zodiac_command/mathUtility.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>


using namespace std;

#define DATA_OUT_OF_RANGE -2000

// The helm command correspond to the angle of the hord board motor with the boat (in degree)
ros::Publisher helmCmd_pub;
ros::Publisher gpsSpeed_pub;
ros::Publisher gpsCourse_pub;
ros::Publisher boatHeading_pub;
ros::Publisher errorCourse_pub;
std_msgs::Float64 helmCmd_msg;

vector<zodiac_command::WaypointMission> waypointLine;
double boatLatitude = DATA_OUT_OF_RANGE;
double boatLongitude = DATA_OUT_OF_RANGE;
double gpsSpeed = DATA_OUT_OF_RANGE;    // m/s
double gpsCourse = DATA_OUT_OF_RANGE;   // degrees
double boatHeading = DATA_OUT_OF_RANGE; // degrees
double desiredCourse = DATA_OUT_OF_RANGE;  // degrees

double I = 0; // integral sum for PIDregulator
double oldCourseError = 0; // courseError at t-1 for PIDregulator

double maxHelmAngle; // degrees
int regulatorType; // 1: sinus, 2: PID
double gainP;
double gainI;
double gainD;
double offsetMotorAngle;
double magneticDeclination; // degrees
double loopRate; // Hz


// in : courseError = boatCourse - desiredCourse in degrees
// out : helmCmd in degrees
double regulatorSinus(const double courseError)
{
    double courseErrorRad = mathUtility::degreeToRadian(courseError);
    if(cos(courseErrorRad) < 0) // Wrong sense because over +/- pi/2 rad
    {
        // Max helm angle in the opposite way
        return mathUtility::sgn(sin(courseErrorRad))*maxHelmAngle;
    }
    else
    {   // Regulation of the helm
        return sin(courseErrorRad)*maxHelmAngle;
    }
}

// courseError = boatCourse - desiredCourse in degrees
// out : helmCmd in degrees
double regulatorPID(const double courseError)
{
    double P = gainP*courseError;
    double D = gainD*(courseError-oldCourseError)*loopRate;
    double helmCmd = P+I+D;

    I = I + gainI*courseError/loopRate;
    oldCourseError = courseError;

    // Anti wind up and max command
    if (abs(helmCmd) > maxHelmAngle)
    {
        helmCmd = mathUtility::sgn(helmCmd)*maxHelmAngle;
        I = 0;
    }

    return helmCmd;
}

// courseError = boatCourse - desiredCourse in degrees
// out : helmCmd in degrees
double regulatorPIDsin(const double courseError)
{
    double P = gainP*courseError;
    double D = gainD*mathUtility::radianToDegree(sin(mathUtility::degreeToRadian(courseError-oldCourseError)))*loopRate;
    double helmCmd = P+I+D;

    I = I + gainI*courseError/loopRate;
    oldCourseError = courseError;

    // Anti wind up and max command
    if (abs(helmCmd) > maxHelmAngle)
    {
        helmCmd = mathUtility::sgn(helmCmd)*maxHelmAngle;
        I = 0;
    }

    return helmCmd;
}

double regulatorSlalom(const double m_nextWaypointLon, const double m_nextWaypointLat,
    const double m_prevWaypointLon, const double m_prevWaypointLat, const double m_VesselLon, const double m_VesselLat)
// regualteur basé sur le papier Slalom
{
  double helmCmd;

  const int earthRadius = 6371000; //meters

  array<double, 3> prevWPCoord = {
      earthRadius * cos(mathUtility::degreeToRadian(m_prevWaypointLat)) * cos(mathUtility::degreeToRadian(m_prevWaypointLon)),
      earthRadius * cos(mathUtility::degreeToRadian(m_prevWaypointLat)) * sin(mathUtility::degreeToRadian(m_prevWaypointLon)),
      earthRadius * sin(mathUtility::degreeToRadian(m_prevWaypointLat))};

  array<double, 3> nextWPCoord = {
      earthRadius * cos(mathUtility::degreeToRadian(m_nextWaypointLat)) * cos(mathUtility::degreeToRadian(m_nextWaypointLon)),
      earthRadius * cos(mathUtility::degreeToRadian(m_nextWaypointLat)) * sin(mathUtility::degreeToRadian(m_nextWaypointLon)),
      earthRadius * sin(mathUtility::degreeToRadian(m_nextWaypointLat))};

  array<double, 3> boatCoord = {
      earthRadius * cos(mathUtility::degreeToRadian(m_VesselLat)) * cos(mathUtility::degreeToRadian(m_VesselLon)),
      earthRadius * cos(mathUtility::degreeToRadian(m_VesselLat)) * sin(mathUtility::degreeToRadian(m_VesselLon)),
      earthRadius * sin(mathUtility::degreeToRadian(m_VesselLat))};

  array<double, 2> param_droite = {
      // coefficent directeur de la droite formee par les deux waypoints
      (nextWPCoord[1] - prevWPCoord[1]) / (nextWPCoord[0] - prevWPCoord[0]),
      // ordonnee a l origine
      prevWPCoord[1] - ((nextWPCoord[1] - prevWPCoord[1]) / (nextWPCoord[0] - prevWPCoord[0])) * prevWPCoord[0]};

  helmCmd = - mathUtility::sawtooth(boatHeading - atan2(param_droite[0] + (1 / 10) * (param_droite[0] * boatCoord[0] + param_droite[1] - boatCoord[1]), 1)) +
    ((1 / 10) * (param_droite[0] * cos(boatHeading) - sin(boatHeading))) / pow(1 + (param_droite[0] + (1 / 10) * (param_droite[0] * boatCoord[0] + param_droite[1] - boatCoord[1])), 2);

  // Anti wind up and max command
  if (abs(helmCmd) > maxHelmAngle)
  {
      helmCmd = mathUtility::sgn(helmCmd)*maxHelmAngle;
  }

  return helmCmd;

}

void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    geometry_msgs::Vector3 linearVelocity = msg->twist.linear;

    if (!isnan(linearVelocity.x) && !isnan(linearVelocity.y))
    {
        gpsSpeed = sqrt(pow(linearVelocity.x,2) + pow(linearVelocity.y, 2));
        gpsCourse = mathUtility::limitAngleRange(mathUtility::radianToDegree(atan2(linearVelocity.x, linearVelocity.y)));
    }
    else{
        gpsSpeed = 0;
        gpsCourse = 0;
    }

    std_msgs::Float64 gpsSpeed_msg;
    std_msgs::Float64 gpsCourse_msg;
    gpsSpeed_msg.data = gpsSpeed;
    gpsCourse_msg.data = gpsCourse;
    gpsSpeed_pub.publish(gpsSpeed_msg);
    gpsCourse_pub.publish(gpsCourse_msg);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double imuHeading = mathUtility::limitAngleRange(-mathUtility::radianToDegree(yaw));
    boatHeading = mathUtility::limitAngleRange(imuHeading - magneticDeclination);

    std_msgs::Float64 boatHeading_msg;
    boatHeading_msg.data = boatHeading;
    boatHeading_pub.publish(boatHeading_msg);
}

void waypointLine_callback(const zodiac_command::WaypointListMission::ConstPtr& msg)
{
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
        ROS_WARN_THROTTLE(5, "No gps fix");
    }
}

void desiredCourse_callback(const std_msgs::Float64::ConstPtr& msg)
{
    desiredCourse = msg->data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "courseRegulator");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ros::Subscriber vel_sub = nh.subscribe("vel", 1, vel_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu", 1, imu_callback);
    ros::Subscriber desiredCourse_sub = nh.subscribe("desired_course", 1, desiredCourse_callback);

    ros::Subscriber waypointLine_sub = nh.subscribe("waypoint_line", 1, waypointLine_callback);
    ros::Subscriber fix_sub = nh.subscribe("fix", 1, fix_callback);



    helmCmd_pub = nh.advertise<std_msgs::Float64>("helm_angle_cmd", 1);
    gpsSpeed_pub = nh.advertise<std_msgs::Float64>("gps_speed", 1);
    gpsCourse_pub = nh.advertise<std_msgs::Float64>("gps_course", 1);
    boatHeading_pub = nh.advertise<std_msgs::Float64>("boat_heading", 1);
    errorCourse_pub = nh.advertise<std_msgs::Float64>("error_course", 1);

    nhp.param<double>("courseRegulator/max_helm_angle", maxHelmAngle, 25);
    nhp.param<int>("courseRegulator/regulator_type", regulatorType, 1);
    nhp.param<double>("courseRegulator/PID/gain_P", gainP, 1);
    nhp.param<double>("courseRegulator/PID/gain_I", gainI, 0);
    nhp.param<double>("courseRegulator/PID/gain_D", gainD, 0);
    nhp.param<double>("courseRegulator/offset_motor_angle", offsetMotorAngle, 0);

    nhp.param<double>("imu/magnetic_declination", magneticDeclination, 0);

    nhp.param<double>("courseRegulator/loop_rate", loopRate, 1);
    ros::Rate loop_rate(loopRate);

    while (ros::ok())
    {
        if ((gpsSpeed != DATA_OUT_OF_RANGE) && (gpsCourse != DATA_OUT_OF_RANGE) &&
            (boatHeading != DATA_OUT_OF_RANGE) && (desiredCourse != DATA_OUT_OF_RANGE) &&
            (boatLatitude != DATA_OUT_OF_RANGE) && (boatLongitude != DATA_OUT_OF_RANGE))
        {
            double errorCourse = mathUtility::limitAngleRange180(boatHeading - desiredCourse);

            std_msgs::Float64 errorCourse_msg;
            errorCourse_msg.data = errorCourse;
            errorCourse_pub.publish(errorCourse_msg);

            switch(regulatorType)
            {
            case 1 : // sinus regulator
                helmCmd_msg.data = regulatorSinus(errorCourse) + offsetMotorAngle;
                break;
            case 2 : // PID regulator
                helmCmd_msg.data = regulatorPID(errorCourse) + offsetMotorAngle;
                break;
            case 3 : // PIDsin regulator
                helmCmd_msg.data = regulatorPIDsin(errorCourse) + offsetMotorAngle;
                break;
            case 4 :
                helmCmd_msg.data = regulatorSlalom(waypointLine.at(1).longitude, waypointLine.at(1).latitude,
                waypointLine.at(0).longitude, waypointLine.at(0).latitude, boatLongitude, boatLatitude) + offsetMotorAngle;
            }
            helmCmd_pub.publish(helmCmd_msg);
        }
        else
            // ROS_WARN_THROTTLE(10, "courseRegulator : waiting for topic");

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
