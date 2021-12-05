#!/usr/bin/env python3

import numpy as np
import rospy
from core.srv import MotionService, WalkService, ServoService, ImuService, ButtonService
import time

def walk_client(n, step, side, ang):
    rospy.wait_for_service('walk_service')
    try:
        a = rospy.ServiceProxy('walk_service', WalkService)
        a(n, step, side, ang)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def motion_client(motion_id):
    rospy.wait_for_service('motion_service')
    try:
        a = rospy.ServiceProxy('motion_service', MotionService)
        a(motion_id)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def servos_client(names, positions):
    rospy.wait_for_service('servo_service')
    try:
        servos_service = rospy.ServiceProxy('servo_service', ServoService)
        servos_service(names, positions)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def imu_client():
    rospy.wait_for_service('imu_service')
    try:
        a = rospy.ServiceProxy('imu_service', ImuService)
        return a()
    except rospy.ServiceException as e:
        print("Service call failed:", e)


# def compare_angles(x1, x2, diff):
#     if x1 > 180:
#         x1 = x1 - 360
#     if x2 > 180:
#         x2 = x2 - 360

#     return x1 - x2 - diff

if __name__ == "__main__":
    num = 10
    stepLength = 36
    sideLength = 0
    rotation = 0.2


    to_rotate_deg = 30

    imu_start = imu_client().x
    imu_end = imu_start - to_rotate_deg
    imu_end %= 360
    while True:
        walk_client(True, 48, 0,0 )
        time.sleep(5)
    while True:
        imu = imu_client()
        print("imu: ", imu.x)
        print("imu end: ", imu_end)
        if np.abs(imu.x - imu_end) < 3:
            break


        if imu.x - imu_end > 0:
            print("Positive")
            walk_client(True, 0, 0, rotation)
        else:
            print("Negative")
            walk_client(True, 0, 0, -rotation)

    walk_client(False, 0, 0, 0)
   # while(True):
    # walk_client(True, stepLength, sideLength, rotation)
    # time.sleep(5)
    # walk_client(False, 0, 0, 0)

    # walk_client(True, 0, 0, 0.2)
    # time.sleep(5)
    # walk_client(False, 0, 0, 0.0)

    # motion_client('test_head')

    #servos_client(["head_yaw"], [0.5])
    #time.sleep(0.5)
    #servos_client(["pelvis"], [-0.5])
    #time.sleep(1)
    #servos_client(["head_yaw", "pelvis"], [0.0, 0.0])



