#!/usr/bin/env python3

import os
import rospy
import json
from rospkg import RosPack

from motion.motion_atom import Motion
from core.srv import MotionService, WalkService


class MotionServer:
    def __init__(self):
        self.motion = Motion()
        self.motion_enabled = False
        self.walk_enabled = False
        self.walk_cycles = 0
        self.last_step = False
        
        self.step_length = 0
        self.side_length = 0
        self.angle = 0


        self.motion_config = None

        self.load_config()
        rospy.logdebug(self.motion_config)

    def load_config(self):
        rp = RosPack()
        core_path = rp.get_path("core")
        config_path = core_path + "/config/motion_config_test.json"
        with open(config_path) as f:
            self.motion_config = json.load(f)

    def handle_walk(self, req):
        walk_enabled = req.walk_enabled
        self.step_length = req.step_length
        self.side_length = req.side_length
        self.angle = req.angle

        self.last_step = self.walk_enabled and not walk_enabled
        self.walk_enabled = walk_enabled
        return self.walk_enabled or self.last_step

    def handle_motion(self, req):
        motion_name = req.motion_name

        if not self.motion_config.get(req.motion_name):
            raise Exception("Incorrect motion name given")
        self.motion.kondo.motionPlay(self.motion_config[motion_name])
        return True

    def motion_server(self):
        rospy.init_node('motion_server')
        s_num = rospy.Service('walk_service', WalkService, self.handle_walk)
        s_str = rospy.Service(
            'motion_service', MotionService, self.handle_motion)
        rospy.loginfo(f"Launched \033[92mwalk_service\033[0m and \033[92mmotion_service\033[0m")
        
        while True:
            self.motion_cycle()
            

    def motion_cycle(self):

        if (self.walk_cycles == 0 and self.walk_enabled):
            self.motion.walk_Initial_Pose()

        if self.last_step:
            print("Last step")
            self.walk_cycles = 0
            self.motion.walk_Cycle(
                self.step_length, self.side_length, self.angle, 99, 100)
            self.motion.walk_Cycle(
                self.step_length, self.side_length, self.angle, 100, 100)
            self.motion.walk_Final_Pose()
        elif self.walk_enabled:
            step_length = self.step_length

            if self.walk_cycles == 0: step_length = step_length / 3.
            if self.walk_cycles == 1: step_length = 2* step_length / 3.

            self.motion.walk_Cycle(
                step_length, self.side_length, self.angle, 4, 100)
            self.walk_cycles += 1

        # print("Walk is enabled: ", self.walk_enabled)
        # print("Last step: ", self.last_step)
        # print("Step length: ", step_length)
        # print("Step side: ", self.side_length)
        # print("Step angle: ", self.angle)
        # print("Walk cycle: ", self.walk_cycles)
        

    # def motion_cycle(self, step_length, side_length, angle, last = False):
    #     number_Of_Cycles = cycle_num
    #     stepLength = step_length
    #     sideLength = side_length
    #     rotation = angle
    #     self.motion.walk_Initial_Pose()
    #     number_Of_Cycles += 1
    #     for cycle in range(number_Of_Cycles):
    #         stepLength1 = stepLength
    #         if cycle == 0:
    #             stepLength1 = stepLength/3
    #         if cycle == 1:
    #             stepLength1 = stepLength/3 * 2
    #         self.motion.walk_Cycle(
    #             stepLength1, sideLength, rotation, cycle, number_Of_Cycles)
    #     self.motion.walk_Final_Pose()


if __name__ == "__main__":
    ms = MotionServer()
    ms.motion_server()
