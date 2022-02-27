#include <string>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>


class Transformer {
  ros::NodeHandle nh_;
  ros::Publisher pub_ ;
  ros::Subscriber sub_;
  //tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  //geometry_msg::Vector3 det_direction;

  public:
  Transformer(const sensor_msgs::CameraInfoConstPtr& info_msg): it(nh_), frame_ids_(frame_ids){
    std :: string image_topic = nh_.resolveName("image");
    cam_model_.fromCameraInfo(info_msg);
    pub_ = nh_.advertise<geometry_msgs::Point>("pub_direction", 100);
    
    sub_ = n.subscribe("pixelPoint", 1000, sendDir); //do name

  
  }
  void sendDir(const geometry_msgs :: Pose2D & msg) const{
   const cv::Point2d & uv_rect;	
    uv_rect.x = msg.x;//
    uv_rect.y = msg.y;//????
   cv::Point3d ray_cv;
   cv::Point3d pt_cv;
   ray_cv = cam_model_.projectPixelTo3(uv_rect);
   pt_cv = cam_model_.projectPixelTo3(uv_rect);////chose what to use
   //publish msgs
   geometry_msgs::Point point;
   point.x = ray_cv.x;
   point.y = ray_cv.y;
   point.z = ray_cv.z;
   pub_.publish(point);
  }


};


int main(){
const sensor_msgs::CameraInfoConstPtr& info_msg;//load camera_info
Transformer transform(const sensor_msgs::CameraInfoConstPtr& info_msg);

return 0;
}