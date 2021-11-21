#!/usr/bin/env python3

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

    def callback(self, rgb_msg):
        frame = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, -1)        
    
        cv2.imshow("frame", frame)
        cv2.waitKey(10)

if __name__ == '__main__':
    rospy.init_node('my_node', anonymous=True)

    image_sub = message_filters.Subscriber('hsv_image', Image)

    pipeline = Pipeline()

    ts = message_filters.ApproximateTimeSynchronizer([image_sub], 10, 0.2)
    ts.registerCallback(pipeline.callback)

    rospy.spin()
