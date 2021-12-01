#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import time
import numpy as np
from core.srv import ModelService, ServoService
# import model

class ExampleFSM:
    def __init__(self):
        self.ball = (None, None)
        self.last_seen_ball = 0
        self.camera_pan = 0
        self.camera_tilt = 0

    def update_ball(self, msg):
        self.ball = (msg.x, msg.y)
        self.last_seen_ball = time.time()
        rospy.loginfo(f"Got the ball: {self.ball[0]}, {self.ball[1]}")
        self.ball_self = self.get_ball_distance()
        rospy.loginfo(f"Got the ball in self coords: {self.ball_self[0]}, {self.ball_self[1]}")


    def get_ball_distance(self):
        rospy.wait_for_service('model_service')
        try:
            get_coords = rospy.ServiceProxy('model_service', ModelService)
            print(self.ball[0], self.ball[1], self.camera_pan, self.camera_tilt)
            response = get_coords(int(self.ball[0]), int(self.ball[1]), self.camera_pan, self.camera_tilt)
            return (response.x, response.y)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def tick(self):
        pass
        
        # # rospy.spinOnce()
        # if (time.time() - self.last_seen_ball <= 1.0):
        #     rospy.loginfo("Go to the ball")
        # else: 
        #     rospy.loginfo("Searching for the ball")

    
    def move_head(self, head_pitch = 0 , head_yaw = 0):
        self.camera_pan = head_yaw
        self.camera_tilt = head_pitch
        self.servos_client(["head_yaw", "head_pitch"], [head_yaw, head_pitch])

    @staticmethod
    def servos_client(names, positions):
        rospy.wait_for_service('servo_service')
        try:
            servos_service = rospy.ServiceProxy('servo_service', ServoService)
            servos_service(names, positions)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

if __name__ == "__main__":
    rospy.init_node("example")
    example = ExampleFSM()
    ball_sub = rospy.Subscriber('ball', Point, example.update_ball)
    example.move_head(0, 0)
    while True:
        example.tick()
        time.sleep(0.1)
