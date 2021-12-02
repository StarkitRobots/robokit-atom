#!/usr/bin/env python3

from numpy.lib.scimath import arctanh, sqrt 
import math
import rospy
import time
import numpy as np
from geometry_msgs.msg import Point
from core.srv import ModelService, ServoService, WalkService


class Basketball:
    step_length = 16
    const_step_length = 1
    side_length = 1
    const_side_length = 1
    angle = 0.1
    const_angle = 1
    
    dead_distance = 0.1

    def __init__(self):
        self.ball = (None, None)
        self.basketcase = (None, None)
        self.ball_self = (None, None)
        self.basketcase_self = (None, None)
        self.last_seen_ball = 0
        self.last_seen_basketcase = 0
        self.camera_pan = 0
        self.camera_tilt = 0
        self.flag_ball  = False
        self.flag_basketcase = False
        self.distance_to_ball = None
        self.distance_to_basketcase = None
        self.basketcase_linia = 0
        self.angle_thinking = 0
        self.ball_coordinates = (0, 0)
        self.basketcase_coordinates = (0, 0)

    def update_ball(self, msg):
        self.ball = (msg.x, msg.y)
        self.last_seen_ball = time().time()
        rospy.loginfo(f"Got the ball: {self.ball[0]}, {self.ball[1]}")
        self.ball_self = self.get_ball_distance()
        self.distance_to_ball = sqrt(self.ball_self[0] ** 2 + self.ball_self[1] ** 2)
        rospy.loginfo(f"Got the ball in self coords: {self.ball_self[0]}, {self.ball_self[1]}")

     def update_basketcase(self, msg):
        self.basketcase = (msg.x, msg.y)
        self.last_seen_basketcase = time().time()
        rospy.loginfo(f"Got the basketcase: {self.basketcase[0]}, {self.basketcase[1]}")
        self.basketcase_self = self.get_basketcase_distance()
        self.distance_to_basketcase = sqrt(self.basketcase_self[0] ** 2 + self.basketcase_self[1] ** 2)
        rospy.loginfo(f"Got the basketcase in self coords: {self.basketcase_self[0]}, {self.basketcase_self[1]}")


    def get_ball_distance(self):
        rospy.wait_for_service("model_service")
        try:
            get_coords = rospy.ServiceProxy("model_service", ModelService)
            rospy.loginfo(self.ball[0], self.ball[1], self.camera_pan, self.camera_tilt)
            response = get_coords(int(self.ball[0]), int(self.ball[1]), self.camera_pan, self.camera_tilt)
            return (response.x, response.y)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

    def get_basketcase_distance(self):
        rospy.wait_for_service("model_service")
        try:
            get_coords = rospy.ServiceProxy("model_service", ModelService)
            rospy.loginfo(self.basketcase[0], self.basketcase[1], self.camera_pan, self.camera_tilt)
            response = get_coords(int(self.basketcase[0]), int(self.basketcase[1]), self.camera_pan, self.camera_tilt)
            return (response.x, response.y)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)        

    def radians_search(frequency):
        res = zip(np.zeros(frequency), np.linspace(-np.pi/2, np.pi/2, frequency)) + zip(np.zeros(np.pi/4), np.linspace(np.pi/2, -np.pi/2, frequency))
        return res + [0, 0]

    def finding_ball(self):
        rospy.loginfo("Start finding ball")
        radians = Basketball.radians_search(5)
        l = []
        for elem in radians:
            self.move_head(elem)
            time.sleep(1)
            self.ball_self = self.get_ball_distance()
            l.append(self.ball_self)
            # choose the mediana of all balls in coordinates of robor and look at dispersy, check massive of all finded bolls

        self.ball_coordinates = np.median(np.array(l), axis = 0)
        if self.ball_coordinates != (None, None):
            self.flag_ball = True

    def finding_basketcase(self):
        rospy.loginfo("Start finding ball")
        radians = Basketball.radians_search(5)
        l = []
        for elem in radians:
            self.move_head(elem)
            time.sleep(1)
            self.basketcase_self = self.get_basketcase_distance()
            l.append(self.basketcase_self)
            # choose the mediana of all balls in coordinates of robor and look at dispersy, check massive of all finded bolls

        self.basketcase_coordinates = np.median(np.array(l), axis = 0)
        if self.basketcase_coordinates != (None, None):
            self.flag_ball = True
            
        
                
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


    def turn_to_ball(self):
        #rotation = np.arctan(self.ball_self[1] / self.ball_self[0])
        rotation = np.arctan(self.ball_coordinates[1] / self.ball_coordinates[0])
        time_rotate = rotation * Basketball.const_angle
        self.walk(True, 0, 0, Basketball.angle)
        time.sleep(time_rotate)
        self.walk(False, 0, 0, 0)

    def turn_to_basketcase(self):
        #rotation = np.arctan(self.basketcase_self[1] / self.basketcase_self[0])
        rotation = np.arctan(self.basketcase_coordinates[1] / self.basketcase_coordinates[0])
        time_rotate = rotation * Basketball.const_angle
        self.walk(True, 0, 0, Basketball.angle)
        time.sleep(time_rotate)
        self.walk(False, 0, 0, 0)


    @staticmethod
    def walk(walk_enabled, step_length, side_length, angle):
        rospy.wait_for_service("walk service")
        try: 
            walk_service = rospy.ServiceProxy('walk service', WalkService)
            walk_service(walk_enabled, step_length, side_length, angle)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def go_to_ball(self, percent_distance):
        time_walk = 3 * percent_distance
        self.walk(True, Basketball.step_length, 0, 0)
        time.sleep(time_walk)
        #self.ball_coordinates

    def go_to_basketcase(self, percent_distance):
        time_walk = 3 * percent_distance
        self.walk(True, Basketball.step_length, 0, 0)
        time.sleep(time_walk)

    def thinking_take(self):
        self.basketcase_linia = ((self.basketcase_coordinates[0]) / (self.basketcase_coordinates[1]))
        self.angle_thinking = np.pi/2 - arctanh(self.basketcase_linia)
        time.sleep(1)
        self.distance = (math.cos(self.angle_thinking)) * sqrt(self.ball_self[0] ** 2 + self.ball_self[1] ** 2)
        if(math.fabs(self.distance) > 0.16):
            #take the ball
            #pox, it won't bring down the holder
            pass
        elif(self.distance < 0):
            #take the ball
            #3 steps right
            pass
        elif(self.distance >= 0):
            #take the ball
            #3 steps left
            pass
            
if __name__ == "__main__":
    rospy.init_node("basketball")
    basketball = Basketball()
    ball_sub = rospy.Subscriber('ball', Point, basketball.update_ball)
    basketcase_sub = rospy.Subscriber('basketcase', Point, basketball.update_basketcase)
    basketball.finding_ball()
    if basketball.flag_ball:
        basketball.flag_ball  = False
        basketball.turn_to_ball()
        basketball.go_to_ball(0.8)
        basketball.finding_ball()
        if basketball.flag_ball:
            basketball.flag_ball  = False
            basketball.turn_to_ball()
            basketball.go_to_ball(1)
            basketball.finding_basketcase()
            if basketball.flag_basketcase:
                basketball.flag_basketcase  = False
                basketball.thinking_take()
                basketball.finding_basketcase()
                if basketball.flag_basketcase:
                    basketball.flag_basketcase  = False
                    basketball.turn_to_basketcase()
                    basketball.go_to_basketcase(0.8)
                    basketball.finding_basketcase()
                    if basketball.flag_basketcase:
                        basketball.flag_basketcase  = False
                        basketball.go_to_basketcase(1)
                        #put the ball into basketcase


