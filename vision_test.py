import numpy as np
import cv2

from sensor_msgs.msg import CameraInfo, Image
import message_filters
import rospy

import vision

#from cv_bridge import CvBridge

class Pipeline:
    def __init__(self):
        self.bgr2rgb = vision.Colorspace_transform({"from" : "RGB", "to" : "BGR"})
        self.rgb2hsv = vision.Colorspace_transform({"from" : "RGB", "to" : "HSV"})

    def callback(self, rgb_msg, camera_info):
        #print("sdcsdcsdc")

        rgb_image = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, -1)
        
        #rgb_channels_correct = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        
        rgb_channels_correct = self.bgr2rgb.work(rgb_image)
        hsv = self.rgb2hsv.work(rgb_channels_correct)

        #rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
    
        cv2.imshow("camera_raw", hsv)
        cv2.waitKey(10)

        #camera_info_K = np.array(camera_info.K).reshape([3, 3])
        #camera_info_D = np.array(camera_info.D)
        #rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)

if __name__ == '__main__':
    rospy.init_node('my_node', anonymous=True)

    image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
    info_sub = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)

    pipeline = Pipeline()

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
    ts.registerCallback(pipeline.callback)

    rospy.spin()
