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

class Transformer {
  ros::NodeHandle nh_;
  ros::Publisher pub_ ;
  ros::Subscriber sub_;
  //tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  //geometry_msg::Vector3 det_direction;

  public:
  Transformer(){


    const std::string url="file://${ROS_HOME}/src/feature/send_message/src/config/ost.yaml";
    const std::string cname="usb_cam";
    camera_info_manager::CameraInfoManager cinfo(nh_,cname,url);
    sensor_msgs::CameraInfo msgCameraInfo;
    msgCameraInfo=cinfo.getCameraInfo();

    std :: string image_topic = nh_.resolveName("image");
    cam_model_.fromCameraInfo(msgCameraInfo);
    pub_ = nh_.advertise<geometry_msgs::Point>("pub_direction", 100);
    
    sub_ = nh_.subscribe("pixel_point", 1000, &Transformer::sendDir,this); //do name

  
  }

  void sendDir(const geometry_msgs :: Pose2D & msg){
    cv::Point2d uv_rect;	
    uv_rect.x = msg.x;//
    uv_rect.y = msg.y;//????
   cv::Point3d ray_cv;
   cv::Point3d pt_cv;
   ray_cv = cam_model_.projectPixelTo3dRay(uv_rect);
   //pt_cv = cam_model_.projectPixelTo3(uv_rect);////chose what to use
   //publish msgs
   geometry_msgs::Point point;
   point.x = ray_cv.x;
   point.y = ray_cv.y;
   point.z = ray_cv.z;
   pub_.publish(point);
  }


};


int main(int argc, char** argv){
ros::init(argc, argv, "tf_pixel2vector");
const sensor_msgs::CameraInfoConstPtr info_msg;//load camera_info
Transformer transform();
     /* while (ros::ok())
    {
        marker_publisher.publish();
        ros :: spinOnce();
    }*/

return 0;
}