/*------------------------------------------------------------------------------
* ROS node "navigation" subscribes to "fix", "vel" and "imu" topics. 
* It publishes "pose" topic.
*
* ------------------------------------------------------------------------------
*/

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf/transform_datatypes.h"


// TODO this node ????

class Navigation
{
public:
    Navigation(){
        // Subscribers
        sub_fix = node.subscribe("fix", 1, &Navigation::updateGpsFix, this);
        sub_vel = node.subscribe("vel", 1, &Navigation::updateGpsVel, this);
        sub_imu = node.subscribe("imu", 1, &Navigation::updateImu, this);

        // Publishers
        pose_pub = node.advertise<geometry_msgs::Pose>("pose", 1);
        twist_pub = node.advertise<geometry_msgs::Twist>("twist", 1);

        // Internal variables
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        pose.position.z = 0;
        pose.position.x = 0;
        pose.position.y = 0;

        tf::Quaternion q = tf::createQuaternionFromRPY(0,0,0);
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();

        meters_coord.x = 0;
        meters_coord.y = 0;
    }

    void updateGpsVel(const geometry_msgs::TwistStamped::ConstPtr& msg){
        gpsvel.header = msg->header;
        gpsvel.twist = msg->twist;
        ROS_DEBUG("gps vel received");
    }

    void updateGpsFix(const sensor_msgs::NavSatFix::ConstPtr& msg){
        gpsfix.header = msg->header;
        gpsfix.status = msg->status;
        gpsfix.latitude = msg->latitude;
        gpsfix.longitude = msg->longitude;
        gpsfix.altitude = msg->altitude;
        gpsfix.position_covariance = msg->position_covariance;
        gpsfix.position_covariance_type = msg->position_covariance_type;
        ROS_DEBUG("gps fix: ([%f], [%f])", gpsfix.longitude, gpsfix.latitude);

        // Passage en coordonnées meters
        // meters_coord = latlon2meters(gpsfix.latitude, gpsfix.longitude);
        meters_coord.x = gpsfix.longitude;
        meters_coord.y = gpsfix.latitude;
    }

    void updateImu(const sensor_msgs::Imu::ConstPtr& msg){
        imu.header = msg->header;
        imu.orientation = msg->orientation;
        imu.orientation_covariance = msg->orientation_covariance;
        imu.angular_velocity = msg->angular_velocity;
        imu.angular_velocity_covariance = msg->angular_velocity_covariance;
        imu.linear_acceleration = msg->linear_acceleration;
        imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
        ROS_DEBUG("imu: ([%f])", tf::getYaw(imu.orientation));

    }

    void sensor_fusion(){
        // TODO : faire la fusion de donnée avec un filtre de kalman

        // position
        pose.position.x = meters_coord.x;
        pose.position.y = meters_coord.y;
        pose.position.z = gpsfix.altitude;
        pose.orientation = imu.orientation;

        // speed : linear
        twist.linear.x = gpsvel.twist.linear.x;
        twist.linear.y = gpsvel.twist.linear.y;
        twist.linear.z = gpsvel.twist.linear.z;

        // speed : angular
        twist.angular = imu.angular_velocity;
    }

    void spin(){

        ros::Rate loop(10);

        while (ros::ok()){

            // call all waiting callbacks
            ros::spinOnce();

                sensor_fusion();

                // publish data
                pose_pub.publish(pose);
                // twist_pub.publish(twist);

            loop.sleep();

        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Subscriber
    ros::Subscriber sub_fix;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_imu;

    // Publishers
    ros::Publisher pose_pub;
    ros::Publisher twist_pub;

    // Internal variables
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped twist;

    sensor_msgs::Imu imu;
    sensor_msgs::NavSatFix gpsfix;
    geometry_msgs::TwistStamped gpsvel;

    Coordinates meters_coord;
};


int main(int argc, char **argv)
{
    // Node initialization
    ros::init(argc, argv, "navigation");

    Navigation navigation;

    navigation.spin();
    return 0;
}
