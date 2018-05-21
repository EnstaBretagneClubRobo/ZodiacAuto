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
#include <zodiac_command/mathUtility.h>


using namespace std;

#define DATA_OUT_OF_RANGE -2000

// The helm command correspond to the angle of the hord board motor with the boat (in degree)
ros::Publisher helmCmd_pub;
std_msgs::Int32 helmCmd_msg;

double gpsSpeed = DATA_OUT_OF_RANGE;    // m/s
double gpsCourse = DATA_OUT_OF_RANGE;   // degrees
double boatHeading = DATA_OUT_OF_RANGE; // degrees
int desiredCourse = DATA_OUT_OF_RANGE;  // degrees

int maxHelmAngle; // degrees
int regulatorType; // 1: sinus, 2: PID
double gainP;
double gainI;
double gainD;
double magneticDeclination; // degrees
double course_estim_speed1; // m/s
double course_estim_speed2; // m/s

// Returns an estimation of the vessel course (angle of the velocity vector).
// boatSpeed < speed_1 : boatCourse = boatHeading
// boatSpeed > speed_2 : boatCourse = gpsCourse
// speed_1 < boatSpeed < speed_2 : boatCourse = combinaison of boatHeading and gpsCourse
// out : estimatedBoatCourse in degrees.
double estimatedBoatCourse()
{
    double boatCourse;

    if (course_estim_speed1 > course_estim_speed2) // Error. Need to be m_speed_1 <= m_speed_2.
    {
        course_estim_speed1 =  course_estim_speed2;
    }

    if (cos(mathUtility::degreeToRadian(boatHeading - gpsCourse)) < 0){
        boatCourse = mathUtility::limitAngleRange(gpsCourse + 180);
        gpsSpeed = -gpsSpeed;
    }
    else{
        boatCourse = gpsCourse;
    }

    if(std::abs(gpsSpeed) < course_estim_speed1)
    {
        return boatHeading;
    }
    else if(std::abs(gpsSpeed) >= course_estim_speed2)
    {
        return boatCourse;
    }
    else // m_speed_1 <= m_VesselSpeed < m_speed_2
    {
        return mathUtility::linearFunctionBetweenAngle(gpsSpeed, course_estim_speed1, 
            course_estim_speed2, boatHeading, boatCourse);
    }
}


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
    //TODO
    return 0;
}


void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    gpsSpeed = 0; // TODO
    gpsCourse = mathUtility::limitAngleRange(0); //TODO
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    double imuHeading = 0; //TODO
    boatHeading = mathUtility::limitAngleRange(imuHeading + magneticDeclination);
}

void desiredCourse_callback(const std_msgs::Int32::ConstPtr& msg)
{
    desiredCourse = msg->data;
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

    nhp.param<int>("courseRegulator/max_helm_angle", maxHelmAngle, 25);
    nhp.param<int>("courseRegulator/regulator_type", regulatorType, 1);
    nhp.param<double>("courseRegulator/PID/gain_P", gainP, 1);
    nhp.param<double>("courseRegulator/PID/gain_I", gainI, 0);
    nhp.param<double>("courseRegulator/PID/gain_D", gainD, 0);

    nhp.param<double>("courseRegulator/course_estimation/speed_1", course_estim_speed1, 0.5);
    nhp.param<double>("courseRegulator/course_estimation/speed_2", course_estim_speed2, 1);
    nhp.param<double>("imu/magnetic_declination", magneticDeclination, 0);

    double loopRate;
    nhp.param<double>("courseRegulator/loop_rate", loopRate, 1);
    ros::Rate loop_rate(loopRate);

    while (ros::ok())
    {
        if ((gpsSpeed != DATA_OUT_OF_RANGE) && (gpsCourse != DATA_OUT_OF_RANGE) && 
            (boatHeading != DATA_OUT_OF_RANGE) && (desiredCourse != DATA_OUT_OF_RANGE))
        {
            switch(regulatorType) 
            {
            case 1 : // sinus regulator
                helmCmd_msg.data = (int) regulatorSinus(estimatedBoatCourse() - desiredCourse);
                break;
            case 2 : // PID regulator
                helmCmd_msg.data = (int) regulatorPID(estimatedBoatCourse() - desiredCourse);
                break;
            }        
            helmCmd_pub.publish(helmCmd_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}