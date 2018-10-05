/*------------------------------------------------------------------------------
* ROS node "control" subscribes to "fix" and "imu" topics.
* It publishes "helm_cmd" topic.
*
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <zodiac_command/mathUtility.h>
#include <tf/transform_datatypes.h>

using namespace std;

#define DATA_OUT_OF_RANGE -2000
#define EARTH_RADIUS 6371000
#define LAT0 48.197444
#define LONG0 -3.021258

// The helm command correspond to the angle of the hord board motor with the boat (in degree)
ros::Publisher helmCmd_pub;
ros::Publisher boatHeading_pub;
ros::Publisher pos_pub;
std_msgs::Float64 helmCmd_msg;

double boatHeading = DATA_OUT_OF_RANGE; // degrees
double boatLatitude = DATA_OUT_OF_RANGE; // degrees
double boatLongitude = DATA_OUT_OF_RANGE; // degrees
double pos[2] = {}; // m
double currents[3] = {}; // {p0, p1, p2}

double maxHelmAngle; // degrees
double offsetMotorAngle;
double magneticDeclination; // degrees
double loopRate; // Hz


void GPS2RefCoordSystem(double lat, double lon)
{
  pos[0] = (M_PI/180)*EARTH_RADIUS*(lon-LONG0)*cos((M_PI/180)*lat);
  pos[1] = (M_PI/180)*EARTH_RADIUS*(lat-LAT0);
}

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg)
{
    if (fix_msg->status.status >= fix_msg->status.STATUS_FIX)
    {
        boatLatitude = fix_msg->latitude;
        boatLongitude = fix_msg->longitude;
        GPS2RefCoordSystem(boatLatitude, boatLongitude);
        geometry_msgs::Vector3 pos_msg;
        pos_msg.x = pos[0];
        pos_msg.y = pos[1];
        pos_pub.publish(pos_msg);
    }
    else
    {
        ROS_WARN_THROTTLE(5, "No gps fix");
    }
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

void currents_callback(const geometry_msgs::Vector3::ConstPtr& cur_msg)
{
    currents[0] = cur_msg->x;
    currents[1] = cur_msg->y;
    currents[2] = cur_msg->z;
}

double sawtooth(const float x)
{
  return fmod(x+M_PI, 2*M_PI)-M_PI;
}

double control()
{
    float x1 = pos[0], x2 = pos[1], x3 = M_PI/180*boatHeading;
    float phat1 = currents[0], phat2 = currents[1], phat3 = currents[2];
    float a = 1;
    float da = 0;
    float b = cos(x1) - x2 + sin(x1);
    float db = (phat1*cos(x3)+phat2) * (cos(x1)-sin(x1)) - (phat1*sin(x3)+phat3);
    float y = sawtooth(atan2(phat1*sin(x3)+phat3, phat1*cos(x3)+phat2) - atan2(b,a));
    float bx = (b*da-a*db) / (pow(a, 2)+pow(b, 2));
    float u = -10*y - bx;
    return u;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ros::Subscriber vel_sub = nh.subscribe("fix", 1, fix_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu", 1, imu_callback);
    ros::Subscriber currents_sub = nh.subscribe("currents", 1, currents_callback);

    helmCmd_pub = nh.advertise<std_msgs::Float64>("helm_angle_cmd", 1);
    boatHeading_pub = nh.advertise<std_msgs::Float64>("boat_heading", 1);

    nhp.param<double>("courseRegulator/max_helm_angle", maxHelmAngle, 25);
    nhp.param<double>("courseRegulator/offset_motor_angle", offsetMotorAngle, 0);
    nhp.param<double>("imu/magnetic_declination", magneticDeclination, 0);
    nhp.param<double>("courseRegulator/loop_rate", loopRate, 1);
    ros::Rate loop_rate(loopRate);

    while (ros::ok())
    {
        if (boatHeading != DATA_OUT_OF_RANGE)
        {
            helmCmd_msg.data = control() + offsetMotorAngle;
            helmCmd_pub.publish(helmCmd_msg);
        }
        // else
            // ROS_WARN_THROTTLE(10, "courseRegulator : waiting for topic");

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
