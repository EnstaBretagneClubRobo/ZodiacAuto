/*------------------------------------------------------------------------------
* ROS node "courseRegulator" subscribes to "vel", "imu" and "desired_course" topics. 
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
#include "std_msgs/Float64.h"
#include <zodiac_command/mathUtility.h>
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
double magneticDeclination; // degrees
double course_estim_speed1; // m/s
double course_estim_speed2; // m/s
double loopRate; // Hz

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

//    if (cos(mathUtility::degreeToRadian(boatHeading - gpsCourse)) < 0){
//        boatCourse = mathUtility::limitAngleRange(gpsCourse + 180);
//        gpsSpeed = -gpsSpeed;
//    }
//    else{
//        boatCourse = gpsCourse;
//    }

    if(std::abs(gpsSpeed) < course_estim_speed1)
    {
        return boatHeading;
    }
    else if(std::abs(gpsSpeed) >= course_estim_speed2)
    {
        return gpsCourse;
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

    nhp.param<double>("courseRegulator/course_estimation/speed_1", course_estim_speed1, 0.5);
    nhp.param<double>("courseRegulator/course_estimation/speed_2", course_estim_speed2, 1);
    nhp.param<double>("imu/magnetic_declination", magneticDeclination, 0);

    nhp.param<double>("courseRegulator/loop_rate", loopRate, 1);
    ros::Rate loop_rate(loopRate);

    while (ros::ok())
    {
        if ((gpsSpeed != DATA_OUT_OF_RANGE) && (gpsCourse != DATA_OUT_OF_RANGE) && 
            (boatHeading != DATA_OUT_OF_RANGE) && (desiredCourse != DATA_OUT_OF_RANGE))
        {
            //double errorCourse = mathUtility::limitAngleRange180(estimatedBoatCourse() - desiredCourse);
            double errorCourse = mathUtility::limitAngleRange180(boatHeading - desiredCourse);

            std_msgs::Float64 errorCourse_msg;
            errorCourse_msg.data = errorCourse;
            errorCourse_pub.publish(errorCourse_msg);

            switch(regulatorType)
            {
            case 1 : // sinus regulator
                helmCmd_msg.data = regulatorSinus(errorCourse);
                break;
            case 2 : // PID regulator
                helmCmd_msg.data = regulatorPID(errorCourse);
                break;
            }
            helmCmd_pub.publish(helmCmd_msg);
        }
        else
            ROS_WARN_THROTTLE(10, "courseRegulator : waiting for topic");

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
