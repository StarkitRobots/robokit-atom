#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>
#include <geometry_msgs/Point.h>

geometry_msgs::Point sensor_pos;

void markerCb(const geometry_msgs::Point& point){
    sensor_pos.x = point.x;
    sensor_pos.y = point.y;
    sensor_pos.z = point.z;
	ROS_INFO("I got a hit!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_publisher");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 100);
    ros :: Subscriber sub_ = n.subscribe("pub_direction", 1000, markerCb); //do name
    visualization_msgs::Marker marker;

    ros::Rate loop_rate(100);
    const double degree = M_PI / 180;
    marker.type = 0;
    marker.id = 0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.header.frame_id = "camera";

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    geometry_msgs :: Point zero ;
    zero.x = 0;
    zero.y = 0;
    zero.z = 0;
    sensor_pos.x = 0;
    sensor_pos.y = 0;
    sensor_pos.z = 0;
    marker.points.push_back (zero);
    marker.points.push_back (sensor_pos);
    
    double angle = 0;
    while (ros::ok())
    {
        marker.points[1] = sensor_pos;
        marker.header.stamp = ros :: Time :: now();
        marker_pub.publish(marker);
        ros :: spinOnce();
        loop_rate.sleep();
    }   
    return 0;
}
