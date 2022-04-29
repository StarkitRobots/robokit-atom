#!/usr/bin/env python3

import os
import rospy
import json
import time
from rospkg import RosPack

from motion.sprint_motion_atom import Motion
from core.srv import SprintMotionService, WalkService, ServoService


class MotionServer:
    def __init__(self):
        self.walk_config = None
        self.motion_config = None
        self.servo_config = None

        self.load_config()
        self.load_walk_config()
        rospy.logdebug(self.walk_config)
        self.motion = Motion(self.walk_config)
        self.motion_enabled = False
        self.walk_enabled = False
        self.walk_cycles = 0
        self.last_step = False

        self.step_length = 0
        self.side_length = 0
        self.angle = 0

        rospy.logdebug(self.motion_config)

    def load_config(self):
        rp = RosPack()
        core_path = rp.get_path("core")

        config_path = core_path + "/config/motion_config_test.json"
        with open(config_path) as f:
            self.motion_config = json.load(f)

    def load_walk_config(self):
        rp = RosPack()
        core_path = rp.get_path("core")
        config_path = core_path + "/config/walk_config.json"
        with open(config_path) as f:
            self.walk_config = json.load(f)

        config_path = core_path + "/config/servo_config.json"
        with open(config_path) as f:
            self.servo_config = json.load(f)

        config_path = core_path + "/config/servo_config.json"
        with open(config_path) as f:
            self.servo_config = json.load(f)

        config_path = core_path + "/config/servo_config.json"
        with open(config_path) as f:
            self.servo_config = json.load(f)

    def handle_walk(self, req):
        walk_enabled = req.walk_enabled
        self.step_length = req.step_length
        self.side_length = req.side_length
        self.angle = req.angle
        self.tilt = req.tilt
        if not self.last_step:
            self.last_step = self.walk_enabled and not walk_enabled
        self.walk_enabled = walk_enabled
        return self.walk_enabled or self.last_step

    def handle_motion(self, req):
        motion_name = req.motion_name

        if not self.motion_config.get(req.motion_name):
            raise Exception("Incorrect motion name given")
        self.motion.kondo.motionPlay(self.motion_config[motion_name])
        return True

    def handle_servo(self, req):
        names = req.names
        positions = req.positions
        start1 = time.perf_counter()
        servoDatas = []

        for name, position in zip(names, positions):
            pos = int(position * 1698 + 7500)
            servoDatas.append(self.motion.kondo.ServoData(self.servo_config[name][0],
                                                          self.servo_config[name][1],
                                                          pos))
        # servoDatas = self.motion.reOrderServoData(servoDatas)

        a = self.motion.kondo.setServoPos(servoDatas, self.motion.frames_per_cycle)

        time1 = time.perf_counter() - start1
        time.sleep(time1)
        return a


    def motion_server(self):
        rospy.init_node('motion_server')
        s_num = rospy.Service('walk_service', WalkService, self.handle_walk)
        s_str = rospy.Service(
            'motion_service', SprintMotionService, self.handle_motion)
        s_serv = rospy.Service('servo_service', ServoService, self.handle_servo)
        rospy.loginfo(f"Launched \033[92mwalk_service\033[0m, \033[92mservo_service\033[0m and \033[92mmotion_service\033[0m")

        while True:
            self.motion_cycle()


    def motion_cycle(self):

        if (self.walk_cycles == 0 and self.walk_enabled):
            self.motion.walk_Initial_Pose()

        if self.last_step:
            print("Last step")
            self.last_step = False
            self.walk_cycles = 0
            self.motion.walk_Cycle(
                self.step_length, self.side_length, self.angle, 99, 100, 0)
            self.motion.walk_Cycle(
                self.step_length, self.side_length, self.angle, 100, 100, 0)
            self.motion.walk_Final_Pose()
        elif self.walk_enabled:
            step_length = self.step_length

            if self.walk_cycles == 0: step_length = step_length / 3.
            if self.walk_cycles == 1: step_length = 2* step_length / 3.

            self.motion.walk_Cycle(
                step_length, self.side_length, self.angle, self.head_tilt)
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
