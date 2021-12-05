#!/usr/bin/env python3

import os
import rospy
import json
import time
from rospkg import RosPack

from arduino.arduino import Serial_arduino
from core.srv import ImuService, ButtonService


class ArduinoServer:
    def __init__(self):
        self.arduino = Serial_arduino()
        self.arduino.init_imu()

    def get_imu(self, par):
        return self.arduino.euler()

    def get_button(self, par):
        return self.arduino.get_button()

    def arduino_server(self):
        rospy.init_node('arduino_server')
        s_imu = rospy.Service('imu_service', ImuService, self.get_imu)
        s_but = rospy.Service(
            'button_service', ButtonService, self.get_button)
        rospy.loginfo(f"Launched \033[92mimu_service\033[0m and \033[92mbutton_service\033[0m")
        rospy.spin()

if __name__ == "__main__":
    ms = ArduinoServer()
    ms.arduino_server()