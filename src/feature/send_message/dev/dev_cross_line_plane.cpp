#include <string>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <camera_info_manager/camera_info_manager.h>




int main (){
 ros::init(argc, argv, "cross_line_plane");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 100);
    ros::Subscriber sub_ = n.subscribe("pub_direction", 1000, markerCb); //do name
    visualization_msgs::Marker marker;

    ros::Rate loop_rate(1);
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