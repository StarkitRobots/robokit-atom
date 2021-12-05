#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import time
import numpy as np
from core.srv import ModelService, WalkService, SprintWalkService
from fiducial_msgs.msg import FiducialTransformArray
from scipy.spatial.transform import Rotation
# import model
import math
from std_msgs.msg import String
import numpy as np
start = 3.40
end = 0.5
critical_degree = 15 * math.pi / 180
lateral_def = 0.2
coeff_pan_forward = 0.03
coeff_y_forward = 0.03
coeff_pan_backward = 0.1
coeff_y_backward = 0.1
queue_size = 5

class Sprint:

    """run sprint competition """

    def __init__(self) -> None:
        self.camera_pan = 0
        self.camera_tilt = 0
        self.my_position = None
        self.rotation_from_camera = None
        self.my_pan = 0
        self.my_tilt = 0
        self.forward_step = 48
        self.backward_step = -36
        self.topic = "/logger"
        self.pub_log = rospy.Publisher(self.topic, String)
        self.pans = np.array([])

    def walk_service(self, enable, step_length = 0, side_length = 0, rotation = 0):
        try:
            rospy.wait_for_service('walk_service')
            self.walk_service_client = rospy.ServiceProxy('walk_service', WalkService)
            self.walk_service_client(enable, step_length, 0, rotation)
        except:
            pass


    def update_aruco_pose(self, msg):
        if (len(msg.transforms) == 0):
            return
        self.my_position = msg.transforms[0].transform
        self.quat_to_euler()
        #rospy.loginfo(f"Got the aruco: {self.my_position.translation}")

    def quat_to_euler(self):
        # add camera fix
        #rospy.loginfo(f"Rotation: {self.my_position.rotation}")
        self.rotation_from_camera = Rotation.from_quat(
            [self.my_position.rotation.x, self.my_position.rotation.y, self.my_position.rotation.z, self.my_position.rotation.w])
        self.my_pan = self.rotation_from_camera.as_euler('xyz')[1]
        self.my_tilt = self.rotation_from_camera.as_euler('xyz')[0]
        self.my_what = self.rotation_from_camera.as_euler('xyz')[2]
        self.update_pans();
        self.pub_log.publish("angels:   " + str(self.my_pan) +"  " +  str(self.my_tilt) + "    "+ str(self.my_what))

    def update_pans(self):
        if (len(self.pans) == queue_size):
            self.pans = np.delete(0, self.pans)
        self.pans = np.append(self.pans, self.my_pan)
        self.my_pan = np.median(self.pans)
    def walking_forward_tick(self):
        # a(n, step, side, ang)
        # except rospy.ServiceException as e:
        # rospy.loginfo("Service call failed:", e)
        rospy.loginfo("walking_forward_tick")


        if (self.my_position == None):
            rospy.loginfo("starting")
            return True
        if (self.my_position.translation.z <= end):
            rospy.loginfo("Stop and go back")
            return False
        elif(self.my_pan > critical_degree):  # 10 degree
            self.walk_service (True, self.forward_step, 0, min(-self.my_pan, -0.004))
            rospy.loginfo("go right")
            rospy.loginfo(f"my pan: {self.my_pan}, {self.my_tilt}")
        elif(self.my_pan < - critical_degree):
            self.walk_service (True, self.forward_step,0,max(self.my_pan, 0.004))
            rospy.loginfo("go left")
            rospy.loginfo(f"my pan: {self.my_pan}, {self.my_tilt}")
        else:
            self.walk_service(True, self.forward_step, 0,0)
            rospy.loginfo(f"my pan: {self.my_pan}, {self.my_tilt}")
            rospy.loginfo("go forward")
        return True

    def walking_backward_tick(self):
        # a(n, step, side, ang)
        # except rospy.ServiceException as e:
        # rospy.loginfo("Service call failed:", e)
        if (self.my_position == None):
            rospy.loginfo("starting")
            return True
        if (self.my_position.translation.z >= start):
            rospy.loginfo("Stop and relax")
            exit()  # fine exit
        elif(self.my_pan > critical_degree):  # 10 degree
            self.walk_service (True, self.backward_step, self.my_pan, self.my_position.translation.y)
            rospy.loginfo("go right")
            rospy.loginfo(f"my pan: {self.my_pan}, {self.my_tilt}")
        elif(self.my_pan < - critical_degree):
            self.walk_service (True, self.backward_step, self.my_pan, self.my_position.translation.y)
            rospy.loginfo("go left")
            rospy.loginfo(f"my pan: {self.my_pan}, {self.my_tilt}")
        else:
            self.walk_service(True, self.backward_step, self.my_pan * coeff_pan_forward,
                        self.my_position.translation.y * coeff_y_forward)
            rospy.loginfo(f"my pan: {self.my_pan}, {self.my_tilt}")
            rospy.loginfo("go forward")


if __name__ == "__main__":
    rospy.init_node("sprint")
    rospy.loginfo("1")
    sprint_node = Sprint()
    rospy.loginfo("1")
    aruco_sub = rospy.Subscriber(
        '/fiducial_transforms', FiducialTransformArray, sprint_node.update_aruco_pose)
    time_to_wait = 0.5
    rospy.loginfo("1")
    while True:
        flag_go_back = sprint_node.walking_forward_tick()

        time.sleep(time_to_wait)
        if (flag_go_back == 0):
            break
            sprint_node.walk_service(False)
            time.sleep(1)
    while True:
        sprint_node.walking_backward_tick()
        time.sleep(time_to_wait)
