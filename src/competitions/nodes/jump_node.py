#!/usr/bin/env python3

import cv2
import numpy as np
import os

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from core.srv import MotionService, WalkService, ServoService, ModelService


def fill_holes(img):
    (h, w) = img.shape

    img_enlarged = np.zeros((h + 2, w + 2), np.uint8)
    img_enlarged[1:h + 1, 1:w + 1] = img

    img_enl_not = cv2.bitwise_not(img_enlarged)
    th, im_th = cv2.threshold(img_enl_not, 220, 255, cv2.THRESH_BINARY_INV)

    im_floodfill = im_th.copy()

    h, w = im_th.shape[:2]
    mask = np.zeros((h + 2, w + 2), np.uint8)

    cv2.floodFill(im_floodfill, mask, (0, 0), 255)
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    im_out = im_th | im_floodfill_inv

    result = im_out[1:h - 1, 1:w - 1]

    return result


def leave_max_connected_component(mask):
    result = np.zeros_like(mask)
    output = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)
    labels_num = output[0]
    labels = output[1]
    stats = output[2]
    sz = stats.shape[0]

    max_area = 0
    max_label = 0

    if (sz == 1):
        success = False

    for label_num in range(1, sz):
        if (stats[label_num, cv2.CC_STAT_AREA] > max_area):
            max_area = stats[label_num, cv2.CC_STAT_AREA]
            max_label = label_num

    result[np.where(labels == max_label)] = 255

    return result


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


def model_client(object, camera, height):
    rospy.wait_for_service('model_service')
    try:
        get_coords = rospy.ServiceProxy('model_service', ModelService)
        response = get_coords(int(object[0]), int(
            object[1]), camera[0], camera[1], height)
        return (response.x, response.y)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


class TripleJump:
    def __init__(self):
        self.line_center = None
        self.head_pitches = [np.pi/6, np.pi/3]
        self.speed = 0.5
        self.step_length = 0

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

    def find_line(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)

        low_th = [63, 56, 97]
        high_th = [88, 97, 153]

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, tuple(low_th), tuple(high_th))

        #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones([5, 5], np.uint8))
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones([5, 5], np.uint8))

        mask = leave_max_connected_component(mask)
        mask = fill_holes(mask)

        mask_3_channels = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cnt, hierarchy = cv2.findContours(mask, 1, 2)

        rect = cv2.minAreaRect(cnt[0])
        box = cv2.boxPoints(rect)

        self.line_center = np.mean(box, axis=0)
        self.line_center_dist = model_client(
            self.line_center, (self.camera_pan, self.camera_tilt), 0)
        print("line_center dist: ", self.line_center_dist)

    def search_line(self):
        lines = []
        for head_pitch in self.head_pitches:
            move_head(head_pitch, 0)
            time.sleep(1)
            if self.line_center_dist != (None, None):
                tmp = self.line_center_dist[0]
                if self.check_line(tmp):
                    lines.append(tmp)
                else:
                    rospy.loginfo(f"Found strange line with x: {tmp}")
            else:
                rospy.loginfo(f"Dont have any lines")
        if lines:
            return np.median(lines)
        else:
            return -1

    @staticmethod
    def check_line(x):
        return x > 0 and x < 1.5

    def tick(self):
        while True:
            distance_to_line = search_line(self)
            if distance_to_line != -1:
                break
        time_to_walk = distance_to_line / self.speed * 0.8
        walk_client(True, self.step_length, 0, 0)
        time.sleep(time_to_walk)
        walk_client(False, 0, 0, 0)

        while True:
            distance_to_line = search_line(self)
            if distance_to_line != -1:
                break
        time_to_walk = distance_to_line / self.speed * 2 * 0.8 
        walk_client(True, self.step_length / 2, 0, 0)
        time.sleep(time_to_walk)
        walk_client(False, 0, 0, 0)

        while True:
            distance_to_line = search_line(self)
            if distance_to_line != -1:
                break
        time_to_walk = distance_to_line / self.speed * 4 * 0.8 
        walk_client(True, self.step_length / 4, 0, 0)
        time.sleep(time_to_walk)
        walk_client(False, 0, 0, 0)



if __name__ == "__main__":

    rospy.init_node("triple_jump")
    triple_jump = TripleJump()
    rospy.Subscriber("/usb_cam/image_raw", Image, triple_jump.find_line)
    rospy.spin()
