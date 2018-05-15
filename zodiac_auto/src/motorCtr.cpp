/*------------------------------------------------------------------------------
* ROS node "motorCtr" subscribes to "cmd" topic which publishes Int32 command
* messages.
* It connects to the Pololu Jrk and sends new targets depending on the
* command received.
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include "jrk.h"
#include "std_msgs/Int32.h"

using namespace std;

int fd;

void cmd_callback(const std_msgs::Int32::ConstPtr& msg)
{
	// For now, the command is directly sent to the Pololu.
    jrkSetTarget(fd, msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motorCtr");
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub = nh.subscribe("cmd", 1000, cmd_callback);

    string path;
    nh.param<string>("path", path, "/dev/ttyACM0");

    // ROS_INFO(path.c_str());
    if ((fd = jrkConnect(path.c_str()) ) != -1)
    {
        ROS_INFO("Jrk connected");

        // ROS_INFO("Jrk test");
        // jrkTest(fd);

        ros::spin();
        close(fd);
    }

    return 0;
}
