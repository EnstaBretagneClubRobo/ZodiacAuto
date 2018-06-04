/*------------------------------------------------------------------------------
* ROS node "motorCtr" subscribes to "cmd" topic which publishes Int32 command
* messages.
* It connects to the Pololu Jrk and sends new targets depending on the
* command received.
* ------------------------------------------------------------------------------
*/

#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include "jrk.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

using namespace std;

int fd;

ros::Publisher feedback_pub;
ros::Publisher motorOn_pub;

std_msgs::Float64 angle_fb_msg;
std_msgs::Bool motorOn_msg;

double map_helmAnge_to_helmTarget(double helm_angle){
    double target = -0.003167*pow(helm_angle, 2) - 0.3416*pow(helm_angle, 2) + 83.99*helm_angle + 2234;
    target = (target < 0) ? 0 : target;
    target = (4095 < target) ? 4095 : target;
    return target;
}

double map_helmFeedback_to_helmAngle(double helm_fb){
    double helm_angle = 1.224*pow(10, -10)*pow(helm_fb, 3) -3.112*pow(10, -7)*pow(helm_fb, 2) + 0.01146*helm_fb - 25;
    return helm_angle;
}

void cmd_callback(const std_msgs::Float64::ConstPtr& msg)
{
    int target = (int) map_helmAnge_to_helmTarget(msg->data);
    jrkSetTarget(fd, target);
    //cout << "target=" << target << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motorCtr");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ros::Subscriber cmd_sub = nh.subscribe("helm_angle_cmd", 1000, cmd_callback);

    feedback_pub = nh.advertise<std_msgs::Float64>("helm_angle_fb", 1000);
    motorOn_pub = nh.advertise<std_msgs::Bool>("helm_motorOn", 1000);

    string path;
    nhp.param<string>("path", path, "/dev/ttyACM0");

    if ((fd = jrkConnect(path.c_str()) ) != -1)
    {
        ROS_INFO("Jrk connected");

        // ROS_INFO("Jrk test");
        // jrkTest(fd);

        ros::Rate loop_rate(1);
        while (ros::ok())
        {
            double helm_fb = jrkGetScalingFeedback(fd);
            angle_fb_msg.data = map_helmFeedback_to_helmAngle(helm_fb);
            feedback_pub.publish(angle_fb_msg);

            bool motorOn = !(0x2 & jrkGetErrorFlagsHalting(fd));
            motorOn_msg.data = motorOn;
            motorOn_pub.publish(motorOn_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }

        close(fd);
    }

    return 0;
}
