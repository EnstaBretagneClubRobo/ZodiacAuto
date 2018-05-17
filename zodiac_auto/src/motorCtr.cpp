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
#include "std_msgs/UInt16.h"

using namespace std;

int fd;
ros::Publisher feedback_pub;
std_msgs::UInt16 fb_msg;

void cmd_callback(const std_msgs::Int32::ConstPtr& msg)
{
	// For now, the command is directly sent to the Pololu.
    jrkSetTarget(fd, msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motorCtr");
    ros::NodeHandle nh;

    ros::Subscriber cmd_sub = nh.subscribe("helm_cmd", 1000, cmd_callback);
    feedback_pub = nh.advertise<std_msgs::UInt16>("helm_fb", 1000);

    string path;
    nh.param<string>("path", path, "/dev/pololu"); // /dev/pololu is an alias to the pololu USB virtual port path.

    if ((fd = jrkConnect(path.c_str()) ) != -1)
    {
        ROS_INFO("Jrk connected");

        // ROS_INFO("Jrk test");
        // jrkTest(fd);

        ros::Rate loop_rate(1);
        while (ros::ok())
        {
            fb_msg.data = jrkGetScalingFeedback(fd);
            feedback_pub.publish(fb_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }

        close(fd);
    }

    return 0;
}
