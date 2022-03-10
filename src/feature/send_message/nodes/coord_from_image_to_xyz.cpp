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
//#include <tf/stamped_transform.h>
//#include <tf/matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include "matrixx.h"
//// https://roboticsbackend.com/ros-param-yaml-format/#Loading_params_from_a_YAML_file
image_geometry::PinholeCameraModel cam_model_;
geometry_msgs::Point point;

geometry_msgs::Point foundCross(geometry_msgs ::Point point, tf::TransformListener &listener)
{

  geometry_msgs::Point point_cross;
  geometry_msgs::PointStamped plane_point1;
  geometry_msgs::PointStamped plane_point2;
  geometry_msgs::PointStamped plane_point3;
  geometry_msgs::PointStamped foot_point;
  tf::StampedTransform transform;
  
  point_cross.x = 0;
  point_cross.y = 0;
  point_cross.z = 0;
// found points that forming plane of floor in camera frame
  listener.waitForTransform("/camera", "/right_foot", ros::Time(0), ros::Duration(0.5)); // found coordinates right_foot frame in camera frame
  listener.lookupTransform("/camera", "/right_foot", ros::Time(0), transform); 

  plane_point1.point.x = transform.getOrigin().x();
  plane_point1.point.y = transform.getOrigin().y();
  plane_point1.point.z = transform.getOrigin().z();

  foot_point.header.frame_id = "/right_foot";
  foot_point.header.stamp = ros::Time::now();

  foot_point.point.x = 0;
  foot_point.point.y = 1;
  foot_point.point.z = 0;
  listener.transformPoint("/camera", ros::Time(0), foot_point, "/right_foot", plane_point2);

  foot_point.point.x = 0;
  foot_point.point.y = 0;
  foot_point.point.z = 1;
  listener.transformPoint("/camera", ros::Time(0), foot_point, "/right_foot", plane_point3);
//  found equation of plane(equation3) and ray(equation1-2)
  std ::vector<double> equation1 = {1 / point.x, 0, -1, 0};
  std ::vector<double> equation2 = {0, 1 / point.y, -1, 0};
  std ::vector<double> equation3(4);

  Matrix<2, 2> minor;
  Matrix<2, 3> minor_mat;

  minor_mat[0][0] = plane_point2.point.x - plane_point1.point.x;

  minor_mat[0][1] = plane_point2.point.y - plane_point1.point.y;

  minor_mat[0][2] = plane_point2.point.z - plane_point1.point.z;

  minor_mat[1][0] = plane_point3.point.x - plane_point1.point.x;

  minor_mat[1][1] = plane_point3.point.y - plane_point1.point.y;

  minor_mat[1][2] = plane_point3.point.z - plane_point1.point.z;

  std ::vector<double> plane_point1_stl(3);
  plane_point1_stl[0] = plane_point1.point.x;
  plane_point1_stl[1] = plane_point1.point.y;
  plane_point1_stl[2] = plane_point1.point.z;
  equation3[3] = 0;

  for (int i = 0; i < 3; ++i)
  {
    minor[0][0] = minor_mat[0][(1 + i) % 3];
    minor[0][1] = minor_mat[0][(2 + i) % 3];
    minor[1][0] = minor_mat[1][(1 + i) % 3];
    minor[1][1] = minor_mat[1][(2 + i) % 3];
    equation3[i] = minor.det();
    equation3[3] -= equation3[i] * plane_point1_stl[i];
  }
  std ::vector<std::vector<double>> mat = {equation1, equation2, equation3};
  Matrix<3, 4> matrix;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      matrix[i][j] = mat[i][j];
    }
  }

  //  found crossPoint
  matrix.gaussMethod();
  
  point_cross.x = -matrix[0][3];
  point_cross.y = -matrix[1][3];
  point_cross.z = -matrix[2][3];

  return point_cross;
}

void sendDir(const geometry_msgs ::Pose2D &msg)
{

  cv::Point2d uv_rect;
  uv_rect.x = msg.x; 
  uv_rect.y = msg.y; 
  cv::Point3d ray_cv;
  cv::Point3d pt_cv;
  ray_cv = cam_model_.projectPixelTo3dRay(uv_rect);
  point.x = ray_cv.x;
  point.y = ray_cv.y;
  point.z = ray_cv.z;
  
}

int main(int argc, char **argv)
{
  ROS_INFO("M");

  ros::init(argc, argv, "tf_pixel2vector");
  const sensor_msgs::CameraInfoConstPtr info_msg; 
  tf::TransformListener listener;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  const std::string url = "file:///home/hua/laba/src/robokit-atom/src/feature/send_message/config/ost.yaml"; 
  const std::string cname = "usb_cam";
  camera_info_manager::CameraInfoManager cinfo(nh_, cname, url);
  sensor_msgs::CameraInfo msgCameraInfo;
  msgCameraInfo = cinfo.getCameraInfo();

  std ::string image_topic = nh_.resolveName("image");
  cam_model_.fromCameraInfo(msgCameraInfo);
  pub_ = nh_.advertise<geometry_msgs::Point>("pub_direction", 100);

  sub_ = nh_.subscribe("pixel_point", 1000, sendDir); 

  while (ros::ok())
  {
    point = foundCross(point, listener);

    pub_.publish(point);

    ros ::spinOnce();
  }

  return 0;
}