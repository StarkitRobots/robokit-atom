import cv2
import numpy as np
import os

import subprocess

def set_camera_settings(flags, prefix = "v4l2-ctl --set-ctrl="):
    for k, v in flags.items():
        ret = os.system(prefix + k + "=" + str(v))
        #print(ret)

def get_camera_settings(flags, prefix = "v4l2-ctl ", command = "--get-ctrl="):
    values = {}

    for k in flags:
        ret = os.popen(prefix + command + k).read()

        values.update({k : int(ret [len(k) + 2:])})
    
    return values

trackbars_window_name = "thresholds&settings"

flags = {"exposure_auto"                  : 1,
         "white_balance_temperature_auto" : 0,
         "exposure_auto_priority"         : 0,
         "backlight_compensation"         : 0,
         "focus_auto"                     : 0}

controls_list = ["white_balance_temperature", "brightness", "contrast", "saturation", "gain", "sharpness", "focus_absolute"]

set_camera_settings(flags)

values = get_camera_settings(controls_list)

#print("values:", values)

cam = cv2.VideoCapture(0)

low_th  = (57, 150, 110)
high_th = (67, 160, 120)

def nothing (x):
    pass

cv2.namedWindow (trackbars_window_name)

cv2.createTrackbar ("l1", trackbars_window_name,   0, 255, nothing)
cv2.createTrackbar ("h1", trackbars_window_name, 255, 255, nothing)
cv2.createTrackbar ("l2", trackbars_window_name,   0, 255, nothing)
cv2.createTrackbar ("h2", trackbars_window_name, 255, 255, nothing)
cv2.createTrackbar ("l3", trackbars_window_name,   0, 255, nothing)
cv2.createTrackbar ("h3", trackbars_window_name, 255, 255, nothing)

# def set_camera_setting(val):
#     i = val % 1000000

#     set_camera_settings({controls_list[i + 1] : val / 1000000})

# for i, control_name in enumerate (controls_list[1:]):
#     cv2.createTrackbar (control_name, trackbars_window_name, values[control_name], 255,
#         lambda val = val * 1000000 + i : set_camera_setting(val))

def set_white_balance(new_val):
    if (new_val >= 2800):
        set_camera_settings({"white_balance_temperature" : i})

cv2.createTrackbar (controls_list[0], trackbars_window_name, values[controls_list[0]], 6500, lambda i : set_white_balance(i))

cv2.createTrackbar (controls_list[1], trackbars_window_name, values[controls_list[1]], 255, lambda i : set_camera_settings({controls_list[1] : i}))
cv2.createTrackbar (controls_list[2], trackbars_window_name, values[controls_list[2]], 255, lambda i : set_camera_settings({controls_list[2] : i}))
cv2.createTrackbar (controls_list[3], trackbars_window_name, values[controls_list[3]], 255, lambda i : set_camera_settings({controls_list[3] : i}))
cv2.createTrackbar (controls_list[4], trackbars_window_name, values[controls_list[4]], 255, lambda i : set_camera_settings({controls_list[4] : i}))
cv2.createTrackbar (controls_list[5], trackbars_window_name, values[controls_list[5]], 255, lambda i : set_camera_settings({controls_list[5] : i}))
cv2.createTrackbar (controls_list[6], trackbars_window_name, values[controls_list[6]], 255, lambda i : set_camera_settings({controls_list[6] : i}))

while (True):    
    _, frame = cam.read()

    frame = np.flip(frame, axis=0)

    l1 = cv2.getTrackbarPos ("l1", trackbars_window_name)
    h1 = cv2.getTrackbarPos ("h1", trackbars_window_name)
    l2 = cv2.getTrackbarPos ("l2", trackbars_window_name)
    h2 = cv2.getTrackbarPos ("h2", trackbars_window_name)
    l3 = cv2.getTrackbarPos ("l3", trackbars_window_name)
    h3 = cv2.getTrackbarPos ("h3", trackbars_window_name)

    low_th  = (l1, l2, l3)
    high_th = (h1, h2, h3)

    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    
    mask = cv2.inRange(lab, low_th, high_th)
    
    mask_3_channels = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    result = np.concatenate((frame, lab, mask_3_channels), axis = 1)
    cv2.imshow ("result", result)
    
    os.system ('clear')    
    print (low_th[0] * 100 / 255,  high_th[0] * 100 / 255,
           low_th[1] - 128, high_th[1] - 128,
           low_th[2] - 128, high_th[2] - 128)
    
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        break
		
cam.release()
cv2.destroyAllWindows()

#ball: (0, 125, 72) (63, 234, 255)
#goals: (64, 6, 77) (129, 117, 193)
#lines: (64, 6, 176) (129, 117, 255)
