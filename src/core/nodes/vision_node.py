#!/usr/bin/env python3

import numpy as np
import cv2
import json

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point

import message_filters
import rospy
import importlib

from common.module import Module
#from pipeline import Pipeline
import vision

class VisionPipeline(Module):
    def __init__(self, params, image_publisher):
        super().__init__(params)
        self.show = True
        self.filters = []
        self.res_pub = image_publisher
        self.load_configuration(params["processors"])

    def apply(self, inp):
        #read, process, write
        # print("Processing...")
        rgb_image = np.frombuffer(inp.data, dtype=np.uint8).reshape(inp.height, inp.width, -1)
        results = [rgb_image]

        for f in self.filters:
            results.append(f.apply(results[-1]))

        bb, success = results[-1]
        if success:
            self.res_pub.publish(self.get_point_from_bb(bb))
        # return results[-1]

    @staticmethod
    def get_point_from_bb(bb):
        (left, _), (right, bottom) = bb
        return Point(left + right, bottom, 0) 

    def load_configuration(self, vision_module_config):
        self.vision_module_name = vision_module_config["name"]

        vision_library = importlib.import_module("vision.filters")
        
        for filter_params in vision_module_config["filters"]:
            #Creating instance of module
            module_name = filter_params["name"]
            module_instance = getattr(vision_library, module_name)
            module = module_instance(filter_params)
            self.filters.append(module)
            rospy.loginfo("Pipeline \033[1m{}\033[0m: Filter \033[92m{}\033[0m added.".format(rospy.get_name(), module_name))

if __name__ == '__main__':
    rospy.init_node("vision")
    vision_config_path = rospy.get_param("~vision_config")
    with open(vision_config_path) as f:
        vision_config = json.load(f)
    image_publisher = rospy.Publisher(vision_config["pub_topic_name"], Point, queue_size=10)
    pipeline = VisionPipeline(vision_config, image_publisher)

    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, pipeline.apply)

    rospy.spin()

