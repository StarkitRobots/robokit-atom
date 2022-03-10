#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pixel_pub");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs :: Pose2D >("pixel_point", 1000);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {

      geometry_msgs ::Pose2D pixel;
      pixel.x = 295;
      pixel.y = 245;
    ROS_INFO("%f, %f", pixel.x, pixel.y);

    chatter_pub.publish(pixel);
    ros::spinOnce();

    //loop_rate.sleep();
  }


  return 0;
}