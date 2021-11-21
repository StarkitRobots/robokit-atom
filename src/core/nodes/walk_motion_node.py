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
        self.motion_config = None

        self.load_config()
        print(self.motion_config)

    def load_config(self):
        rp = RosPack()
        core_path = rp.get_path("core")
        config_path = core_path + "/config/motion_config_test.json"
        with open(config_path) as f:
            self.motion_config = json.load(f)

    def handle_walk(self, req):
        cycle_num = req.cycle_num
        step_length = req.step_length
        side_length = req.side_length
        angle = req.angle
        try:
            self.motion_cycle(cycle_num, angle)
        except:
            return False
        #print('walk_service called:', cycle_num, step_length, side_length, angle)
        return True

    def handle_motion(self, req):
        motion_name = req.motion_name

        if not self.motion_dict.get(req.motion_name):
            raise Exception("Incorrect motion name given")

        try:
            self.motion.kondo.motionPlay(self.motion_config[motion_name])
        except:
            return False

        return True

    def motion_server(self):
        rospy.init_node('motion_server')
        s_num = rospy.Service('walk_service', WalkService, self.handle_walk)
        s_str = rospy.Service(
            'motion_service', MotionService, self.handle_motion)
        print('ready for motion')
        rospy.spin()

    def motion_cycle(self, cycle_num, step_length, side_length, angle):
        number_Of_Cycles = cycle_num
        stepLength = step_length
        sideLength = side_length
        rotation = angle
        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if cycle == 0:
                stepLength1 = stepLength/3
            if cycle == 1:
                stepLength1 = stepLength/3 * 2
            self.motion.walk_Cycle(
                stepLength1, sideLength, rotation, cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()


if __name__ == "__main__":
    ms = MotionServer()
    ms.motion_server()
