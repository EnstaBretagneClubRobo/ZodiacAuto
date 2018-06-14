/*------------------------------------------------------------------------------
* ROS node "localisation" subscribes to "fix" topic which publishes NavSatFix
* messages, and publish a Pose message on the "state" topic, which contains the
* position.
* ------------------------------------------------------------------------------
*/

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/NavSatFix.h"

ros::Publisher state_pub;
geometry_msgs::Pose pose_msg;

void pos_callback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg)
{
    if (fix_msg->status.status >= fix_msg->status.STATUS_FIX)
    {
        pose_msg.position.x = fix_msg->latitude;
        pose_msg.position.y = fix_msg->longitude;
		// For now, publish directly the latitude and longitude without any conversion.
        state_pub.publish(pose_msg);
    }
    else
        ROS_WARN_THROTTLE(5, "No gps fix");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localisation");
    ros::NodeHandle nh;

    ros::Subscriber pos_sub = nh.subscribe("fix", 1000, pos_callback);
    state_pub = nh.advertise<geometry_msgs::Pose>("state", 1000);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        geometry_msgs::Pose msg;
        state_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
