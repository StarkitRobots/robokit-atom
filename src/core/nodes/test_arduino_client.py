#!/usr/bin/env python3

import rospy
from core.srv import ImuService, ButtonService
import time

def imu_client():
    rospy.wait_for_service('imu_service')
    try:
        a = rospy.ServiceProxy('imu_service', ImuService)
        print(a())
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def button_client():
    rospy.wait_for_service('button_service')
    try:
        a = rospy.ServiceProxy('button_service', ButtonService)
        print(a())
    except rospy.ServiceException as e:
        print("Service call failed:", e)




if __name__ == "__main__":
    
    imu_client()
    time.sleep(1)
    imu_client()
    time.sleep(1)
    button_client()
    time.sleep(1)


        