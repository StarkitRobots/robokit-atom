#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <string>

class MarkerPublisher{
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;
    geometry_msgs::Pose default_pose;
     ros::Rate loop_rate;
    public:

    MarkerPublisher(int argc, char **argv): loop_rate(30){


        marker_pub = n.advertise<visualization_msgs::Marker>("marker", 100);
      //  ros::Rate loop_rate(30);
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
        default_pose.position.x = 0;
        default_pose.position.y = 0;
        default_pose.position.z = 0;
        default_pose.orientation.x = 0.0;
        default_pose.orientation.y = 0.0;
        default_pose.orientation.z = 0.0;
        default_pose.orientation.w = 1.0;

    }
    void setting_pub(int type, int id, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color, std_msgs::Header header){
    marker.type = type;
    marker.id = id;
    marker.scale = scale;
    marker.header = header;
    marker.color = color;
    }
    void publish(geometry_msgs::Pose pose ){
        marker.pose = pose;
        //marker.header.stamp = ros :: Time :: now();
        marker_pub.publish(marker);
    }
    void publish(){
        marker.pose = default_pose;
        //ros::Time::init();
        //marker.header.stamp = ros :: Time :: now();
        marker_pub.publish(marker);
    
        loop_rate.sleep();
    }
    
};
int main(int argc, char **argv)
{
            ros::init(argc, argv, "marker_publisher");


    const double degree = M_PI / 180;
    // robot state
    double angle = 0;
    // message declarations
    MarkerPublisher marker_publisher(argc, argv);
       while (ros::ok())
    {
        marker_publisher.publish();
        ros :: spinOnce();
    }
    return 0;
}
