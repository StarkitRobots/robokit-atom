#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_publisher");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 100);
    ros::Rate loop_rate(30);
    visualization_msgs::Marker marker;
    const double degree = M_PI / 180;
    marker.type = 0;
    marker.id = 0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.header.frame_id = "trunk";

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // robot state
    double angle = 0;
    // message declarations
    while (ros::ok())
    {
        marker.header.stamp = ros :: Time :: now();
        marker_pub.publish(marker);
        ros :: spinOnce();
        loop_rate.sleep();
    }   
    return 0;
}
