/*------------------------------------------------------------------------------
* ROS node "depth_publisher" connects to the Tritech Micron Echocounder and
* publishes the depth of the seabed on the topic "depth".
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include "serial/serial.h"
#include <iostream>
#include "std_msgs/Float64.h"

using namespace std;

serial::Serial ser;

ros::Publisher depth_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    depth_pub = nh.advertise<std_msgs::Float64>("depth", 1000);

    string port;
    nhp.param<string>("port", port, "/dev/ttyUSB0");

    try
    {
	ser.setPort(port);
	ser.setBaudrate(9600);
	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	ser.setTimeout(to);
	ser.open();
    }
    catch (serial::IOException& e)
    {
	ROS_ERROR_STREAM("Unable to open sonar port");
	return -1;
    }

    if (ser.isOpen())
    {
	ROS_INFO_STREAM("Serial port of the sonar initialized");
    }
    else return -1;


    ros::Rate loop_rate(2);

    while (ros::ok())
    {	
//        depth_pub.publish(depth_msg);
        ros::spinOnce();

        if(ser.available()){
            ROS_DEBUG_STREAM("Reading from serial port");
	    string msg = ser.read(ser.avaible());
	    string depth = msg.substr(6,9);
            std_msgs::Float64 depth_msg;
            depth_msg.data = depth;
	    ROS_DEBUG_STREAM("Read: " << depth_msg.data);
            depth_pub.publish(depth_msg);
        }
        loop_rate.sleep();

    }
    return 0;
}
