/*------------------------------------------------------------------------------
* ROS node "pololu" subscribes to "cmd" topic which publishes Int32 command
* messages.
* It connects to the Pololu Maestro and sends new targets depending on the
* command received.
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include "maestro.h"
#include "std_msgs/Int32.h"

using namespace std;

int fd, channel;

void cmd_callback(const std_msgs::Int32::ConstPtr& msg)
{
	// For now, the command is directly sent to the Pololu.
    maestroSetTarget(fd, channel, msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pololu");
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub = nh.subscribe("cmd", 1000, cmd_callback);

    string path;
    nh.param<string>("path", path, "/dev/ttyACM0");
    nh.param<int>("channel", channel, 1);

    if ((fd = maestroConnect(path.c_str())) != -1)
    {
        ROS_INFO("Maestro connected");

        // int pos = maestroGetPosition(fd, channel);
        // ROS_INFO("Position : %d", pos);
        // ROS_INFO("Setting to position : %d", pos+100);
        // maestroSetTarget(fd, channel, pos+100);

        ros::spin();
        close(fd);
    }

    return 0;
}

