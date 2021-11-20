"""
The Zhang-Suen Thinning Algorithm
Designed by A.Babaev for Phystech Liceum 2020
to be used at OpenMV smart camera

Usage:

iterations_number = zhangSuen(img, rows, columns, iterations_limit)

img - binary Image object
rows - number of pixel rows in frame
columns - number of pixels in one row
iterations_limit - Typical number from 10 to 20. If limit if very big
                   then iterations will be stopped automatically.
                   Actual number of iterations will be returned

This algorithm is used for thinning binary image. Two steps will be subsequently applied to the image.
Here is the order of eight neighbors of P1 arranged into a clockwise order:

 P9 P2 P3
 P8 P1 P4
 p7 P6 P5

P2+P3+P4+P5+P6+P7+P8+P9 = the number of non-zero pixel neighbours of P1
s = the number of transitions from 0 to 1, (0 -> 1) in the sequence P2,P3,...,P8,P9,P2.

Step 1:
All pixels are tested and pixels satisfying all the following conditions (simultaneously) are
just noted at this stage.

Condition 0: The pixel P1 is set to 1 and has eight neighbours
Condition 1: P2 * P4 * P6 = 0
Condition 2: P4 * P6 * P8 = 0
Condition 3: 2 < = P2+P3+P4+P5+P6+P7+P8+P9 < = 6
Condition 4: s = 1

Step 2:
After iterating over one line of image and collecting all the pixels satisfying
all step 1 conditions, the noted pixels from previous line are set to 0.

"""

@micropython.viper
def zhangSuen(img: object, rows: int, columns: int, iterations_limit: int)->int:
    c = [[],[]]
    iterations = 0
    while(True):
        iterations += 1
        bank_num = 1
        changes = 0
        for x in range(1, rows - 1):
            for y in range(1, columns - 1):
                xw = x * 160
                if int(img[y + xw]) == 1:
                    x_1w = (x - 1) * 160
                    y_1 = y - 1
                    x1w = (x + 1) * 160
                    y1 = y + 1
                    P2 = img[y + x_1w]
                    P4 = img[y1 + xw]
                    P6 = img[y + x1w]
                    P8 = img[y_1 + xw]
                    P46 = P4 * P6
                    if int(P2 * P46) == 0:
                        if int(P46 * P8) == 0:
                            P3 = img[y1 + x_1w]
                            P5 = img[y1 + x1w]
                            P7 = img[y_1 + x1w]
                            P9 = img[y_1 + x_1w]
                            if 2 <= int(P2+P3+P4+P5+P6+P7+P8+P9) <= 6:
                                m = [(P2,P3),(P3,P4),(P4,P5),(P5,P6),
                                     (P6,P7),(P7,P8),(P8,P9),(P9,P2)]
                                transitions = 0
                                for s in m:
                                    if s == (0,1): transitions += 1
                                if int(transitions) == 1:
                                    c[bank_num].append((x,y))
                                    changes += 1
            bank_num = 1 - bank_num
            for x0,y0 in c[bank_num]: img[int(y0) + int(x0) * 160] = 0
            c[bank_num].clear()
        bank_num = 1 - bank_num
        for x0,y0 in c[bank_num]: img[int(y0) + int(x0) * 160] = 0
        c[bank_num].clear()
        if changes == 0: break
        if iterations >= iterations_limit: break
    return iterations


if __name__=="__main__":

    import sensor, image, time, pyb

    sensor.reset()
    sensor.set_framesize(sensor.QQVGA)
    sensor.set_pixformat(sensor.RGB565)
    sensor.skip_frames(time = 2000)
    clock = time.clock()

    white_threshold = (43, 100, -35, 0, -99, 43) # L A B

    img = sensor.snapshot().lens_corr(strength = 1.45, zoom = 1.0)
    #pyb.delay(2000)
    img.binary([white_threshold], to_bitmap=True)
    clock.tick()
    #img.to_grayscale()
    #print(img.compressed_for_ide(), end = "")
    #pyb.delay(2000)
    img.to_bitmap()
    iterations = zhangSuen(img, 120, 160, 10)
    #img.to_grayscale()
    #print(img.compressed_for_ide(), end = "")
    print('time =', clock.avg())
    print('iterations =', iterations)

