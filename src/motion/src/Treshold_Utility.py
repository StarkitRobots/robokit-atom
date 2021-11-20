# Single Color RGB565 Blob Tracking Example
#
# This example shows off single color RGB565 tracking using the OpenMV Cam.

import sensor, image, time, math, pyb

colorN = 1 # 0 for orange, 1 for blue, 2 for yellow, 3 or 5 for white, 4 for green

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...


th_index = ["orange ball", "blue posts", "yellow posts", "white posts", "green field", "white marking"]
thresholds ={
            "orange ball":      [48, 87, 4, 63, 39, 79],
            "blue posts":		[11, 49, 3, 45, -61, 1],
            "yellow posts":		[57, 100, -25, -3, 14, 76],
            "white posts":		[46, 100, -17, 23, -20, 15],
            "green field":		[5, 71, -65, -16, -5, 58],
            "white marking":	[60, 100, -37, 4, -19, 13]
            }

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 500)
sensor.set_auto_exposure(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time = 500)
#sensor.set_auto_gain(False, gain_db=0 ) # 9.172951)
#sensor.skip_frames(time = 500)
#sensor.set_auto_exposure(False, exposure_us=2000) #6576)
#sensor.skip_frames(time = 500)
sensor.set_auto_whitebal(False, rgb_gain_db = (-6.0, -3.0, 3.0))

clock = time.clock()


while(True):
    clock.tick()
    img = sensor.snapshot().lens_corr(strength = 1.45, zoom = 1.0)
    for blob in img.find_blobs([thresholds[th_index[colorN]]], pixels_threshold=50, area_threshold=50, merge=True):
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=(255,0,0))
            img.draw_line(blob.major_axis_line(), color=(0,255,0))
            img.draw_line(blob.minor_axis_line(), color=(0,0,255))
        # These values are stable all the time.
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        #img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)
    pyb.delay(1000)
    img2 = img.binary([thresholds[th_index[colorN]]],to_bitmap=True, copy=True)
    print(img2.compressed_for_ide(), end="")
    pyb.delay(1000)
    print(clock.fps())
