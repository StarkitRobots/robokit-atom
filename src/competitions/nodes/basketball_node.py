#!/usr/bin/env python3

import rospy
import time
import numpy as np
from geometry_msgs.msg import Point
from core.srv import ModelService, ServoService


class Basketball:
    def __init__(self):
        self.ball = (None, None)
        self.last_seen_ball = 0
        self.camera_pan = 0
        self.camera_tilt = 0
    def update_ball(self, msg):
        self.ball = (msg.x, msg.y)
        self.last_seen_ball = time().time()
        rospy.loginfo(f"Got the ball: {self.ball[0]}, {self.ball[1]}")
    def finding_ball(self):
        rospy.loginfo("Start finding ball1")
        rospy.wait_for_service("servo_service")
        current_coords_ball = self.ball
        try:
            rospy.loginfo("Start finding ball2")
            set_servo = rospy.ServiceProxy("servo_service", ServoService)
            radians_search = [[0, np.linspace(-np.pi, np.pi, 5)[i]] for i in range(5)] + [[np.pi/4, np.linspace(np.pi, -np.pi, 5)[i]] for i in range(5)]
            for elem in radians_search:
                set_servo(["head_pitch", "head_yaw"], elem)
                if self.ball != (None, None):
                    rospy.loginfo(f"Got the ball: {self.ball[0]}, {self.ball[1]}")
                time.sleep(1)
        except rospy.ServiceException as e:
             rospy.loginfo("Service call failed: %s"%e)
                
            
            
        
    def get_ball_distance(self):
        rospy.wait_for_service("model_service")
        try:
            get_coords = rospy.ServiceProxy("model_service", ModelService)
            rospy.loginfo(self.ball[0], self.ball[1], self.camera_pan, self.camera_tilt)
            response = get_coords(int(self.ball[0]), int(self.ball[1]), self.camera_pan, self.camera_tilt)
            return (response.x, response.y)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            
    def tick(self):
        if (time.time() - self.last_seen_ball <= 1.0):
            rospy.loginfo("Go to the ball")
        else:
            rospy.loginfo("Searching for the ball")
            
if __name__ == "__main__":
    rospy.init_node("basketball")
    basketball = Basketball()
    ball_sub = rospy.Subscriber('ball', Point, basketball.update_ball)
    rospy.loginfo(basketball.ball)
    basketball.finding_ball()
    rospy.loginfo(basketball.ball)
    #while True:
     #   basketball.tick()
      #  time.sleep(0.1)
