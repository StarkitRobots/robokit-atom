#!/usr/bin/env python3

import rospy
# from robot_model import RobotModel
from core.srv import ModelService, ModelServiceResponse 
import model
from model.robot_model import RobotModel
# import vision

def self_from_image(pixel_x, pixel_y, camera_pan = 0, camera_tilt = 0):
    robot_model = RobotModel()
    robot_model.update_camera_pan_tilt(camera_pan, camera_tilt)
    return robot_model.image2self(pixel_x, pixel_y)


def handle_self_from_image(req):
    return self_from_image(req.pixel_x, req.pixel_y, req.camera_pan, req.camera_tilt, req.height)



if __name__ == "__main__":
    rospy.init_node("model_service_server")
    
    robot_model = RobotModel()
    model_server = rospy.Service("model_service", ModelService, handle_self_from_image)
    rospy.loginfo(f"Launched \033[92mmodel_service\033[0m")
    rospy.spin()