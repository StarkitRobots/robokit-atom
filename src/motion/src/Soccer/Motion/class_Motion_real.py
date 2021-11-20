#  Walking engine for Starkit Kondo Atom
#  Copyright STARKIT Soccer team of MIPT

import sys
import math, time, json, cv2

sys.path.append("/home/kondo/kondo_atom")
from show_mask import show_mask_and_annotated_objects

#current_work_directory = os.getcwd()
#current_work_directory = current_work_directory.replace('\\', '/')
#if sys.version != '3.4.0':
#    current_work_directory += '/'
#    with open("simulator_lib_directory.txt", "r") as f:
#        simulator_lib_directory = f.read()
#    simulator_lib_directory = simulator_lib_directory.replace('\\', '/')
#    sys.path.append(simulator_lib_directory)
#    import random
#    import sim, threading
#else:
#    import starkit
    #sys.path.append('/')

#sys.path.append( current_work_directory + 'Soccer/')
#sys.path.append( current_work_directory + 'Soccer/Motion/')
#sys.path.append( current_work_directory + 'Soccer/Vision/')
#sys.path.append( current_work_directory + 'Soccer/Localisation/')
#sys.path.append( current_work_directory + 'Soccer/Localisation/PF/')

#from class_Motion import *
from class_Motion import Motion1
from ball_Approach_Steps_Seq import *

def uprint(*text):
    #with open(current_work_directory + "Soccer/log/output.txt",'a') as f:
    #    print(*text, file = f)
    print(*text )

class Motion_real(Motion1):

    def __init__(self, glob, vision, button):
        super().__init__(glob, vision, button)
        self.sensor = cv2.VideoCapture(0)

        

        self.sensor.set(3,320)
        self.sensor.set(4,240)

        

    def head_Tilt_Calibration(self):
            # Калибрация Наклона камеры. Установить мяч на расстоянии (100 см)по низу мяча.
            # Полученная величина наклона камеры эквивалентна (69) градуса от вертикали.
            # Вторая позиция головы 23 градуса к вертикали отличается от первой на 1155

        if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
            uprint(' head_tilt_calibr')
            i= 400
            a = True
            self.kondo.setUserParameter(19,0)
            while(a):
                if (i < -1500) : a=False
                #clock.tick()
                i=i-1
                uprint ('i =', i)
                b=self.kondo.setUserParameter(20,i)
                for j in range(5):
                    
                    #img = self.sensor.snapshot().lens_corr(strength = 1.45, zoom = 1.0)
                    ret, img = self.sensor.read()
                    cv2.imwrite()
                    img1 = cv2.flip(img1, -1)
                    for blob in img.find_blobs([self.vision.TH['orange ball']['th']], pixels_threshold=20, area_threshold=20, merge=True):
                        #if blob.roundness() > 0.5:
                        img.draw_rectangle(blob.rect())
                        img.draw_cross(blob.cx(), blob.cy())
                        uprint('blob.cy() =', blob.cy())
                        #if (blob.y()+ blob.h()) <=120 : a=False
                        if blob.cy() <=120 : a=False

        else:
            import reload as re
            #import cv2
            i= 200
            a = True
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , 0*self.FACTOR[21], self.sim.simx_opmode_oneshot) #head pan to zero
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , 0*self.FACTOR[22], self.sim.simx_opmode_oneshot) #head tilt to zero
            while(a):
                if (i < -1500) : a=False
                #clock.tick()
                i=i-1
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , i * self.TIK2RAD * self.FACTOR[22], self.sim.simx_opmode_oneshot)
                self.sim_simxSynchronousTrigger(self.clientID)
                img1 = self.vision_Sensor_Get_Image()
                img = re.Image(img1)
                for blob in img.find_blobs([self.vision.TH['orange ball']['th']],
                                            pixels_threshold=self.vision.TH['orange ball']['pixel'],
                                            area_threshold=self.vision.TH['orange ball']['area'],
                                            merge=True):
                    img.draw_rectangle(blob.rect())
                    if blob.cy()==120 : a=False
                self.vision_Sensor_Display(img.img)
        self.neck_calibr = i
        self.neck_play_pose = int(self.neck_calibr - 120 * self.params['APERTURE_PER_PIXEL_VERTICAL'] / 0.03375)
        data = {"neck_calibr": self.neck_calibr, "neck_play_pose": self.neck_play_pose}
        if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
            with open(self.glob.current_work_directory + "Init_params/Real/Real_calibr.json", "w") as f:
                json.dump(data, f)
        else:
            with open(self.glob.current_work_directory + "Init_params/Sim/" + self.glob.SIM_OPTION + "/Sim_calibr.json", "w") as f:
                json.dump(data, f)
        uprint( self.get_course_and_distance_to_ball(blob))

    def seek_Ball(self, fast_Reaction_On):                  # seeks ball in 360 dergree one time
        a, course, distance = self.seek_Ball_In_Pose(fast_Reaction_On)
        if a == True: return a, course, distance
        else:
            target_course1 = self.euler_angle[0] + math.pi
            self.turn_To_Course(target_course1)
            a, course, distance = self.seek_Ball_In_Pose(fast_Reaction_On)
            if a == True: return a, course, distance
            else:
                target_course = target_course1 -170
                self.turn_To_Course(target_course)
                return False, 0, 0

    def seek_Ball_In_Pose(self, fast_Reaction_On, penalty_Goalkeeper = False, with_Localization = True):
        self.local.correct_yaw_in_pf()
        if self.robot_In_0_Pose == False:
            if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
                self.kondo.motionPlay(1)
                time.sleep(0.5)
                #self.pyb.delay(500)
            else:
                self.simulateMotion(name = 'Initial_Pose')
            self.robot_In_0_Pose = True
        variants = []
        # U19 - Шея поворот
        # U20 - Шея Наклон
        c = self.neck_play_pose
        head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),
                     (-2667, c-700),(-1333, c-700), (0, c-700), (1333,c-700),(2667, c-700),
                    (-2667, c-1400), (-1333, c-1400), ( 0, c-1400), (1333, c-1400), (2667, c-1400)]
        #head_pose_seq = [2,7,12,13,8,3,1,6,11,10,5,0,4,9,14,2]
        head_pose_seq = [2,7,12,11,6,8,13,14,9,4,3,10,5,0,1,2]
        if penalty_Goalkeeper: head_pose_seq = [2,7,12,11,6,8,13]
        for i in range(len(head_pose_seq)):
            if not(fast_Reaction_On == True and i == 0):
                x = head_pose[head_pose_seq[i]]
                self.neck_pan = x[0]
                self.neck_tilt = x[1]
            if not self.falling_Test() == 0:
                self.local.quality =0
                if self.falling_Flag == 3: uprint('STOP!')
                else: uprint('FALLING!!!', self.falling_Flag)
                return False, 0, 0, [0, 0]
            if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
                self.kondo.setUserParameter(19,self.neck_pan)
                time.sleep(0.2)
                #self.pyb.delay(200)
                self.kondo.setUserParameter(20,self.neck_tilt)
                time.sleep(0.4)
                #self.pyb.delay(400)
            else:
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , self.neck_pan * self.TIK2RAD * self.FACTOR[21], self.sim.simx_opmode_oneshot)   # Шея поворот
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , self.neck_tilt * self.TIK2RAD * self.FACTOR[22], self.sim.simx_opmode_oneshot)  # Шея Наклон
                for j in range(20):
                    self.sim_simxSynchronousTrigger(self.clientID)
            self.refresh_Orientation()
            a, course, dist, blob = self.seek_Ball_In_Frame(with_Localization)
            #print("in frame: ", dist, course)
            if a == True: variants.append ((course, dist)) # *1000
            if fast_Reaction_On == True and a== True: break
        course = 0
        distance = 0
        if len(variants)>0:
            for i in range (len(variants)):
                course = course + variants[i][0]
                distance = distance + variants[i][1]
            course  = course /len(variants)
            distance = distance /len(variants)
            self.neck_pan =int( - course/ self.TIK2RAD)
            D = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK']- self.params['DIAMETER_OF_BALL']/2
            E = (2*distance*D - math.sqrt(4*distance**2*D**2 - 4*(distance**2-self.params['HEIGHT_OF_NECK']**2)*(D**2 -self.params['HEIGHT_OF_NECK']**2)))/(2*(D**2-self.params['HEIGHT_OF_NECK']**2))
            alpha = math.atan(E)
            alpha_d = math.pi/2 - alpha
            self.neck_tilt = int((-alpha_d)/self.TIK2RAD + self.neck_calibr)
            #uprint('self.neck_pan =', self.neck_pan, 'self.neck_tilt =', self.neck_tilt)
            if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
                self.kondo.setUserParameter(19,self.neck_pan)
                time.sleep(0.2)
                #self.pyb.delay(200)
                self.kondo.setUserParameter(20,self.neck_tilt)
                time.sleep(0.4)
                #self.pyb.delay(400)
            else:
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                             self.jointHandle[21] , self.neck_pan * self.TIK2RAD * self.FACTOR[21], self.sim.simx_opmode_oneshot)   # Шея поворот
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                             self.jointHandle[22] , self.neck_tilt * self.TIK2RAD * self.FACTOR[22], self.sim.simx_opmode_oneshot)  # Шея Наклон
                for j in range(16):
                    self.sim_simxSynchronousTrigger(self.clientID)
            self.refresh_Orientation()
            #a, course, dist, speed = self.detect_Ball_Speed()
            if with_Localization: self.local.localisation_Complete()
            #self.local.pf_update()
            if a == True or (a== False and distance !=0):
                #course_global = course + self.euler_angle[0] + self.neck_pan * 0.03375
                course_global_rad = course + self.glob.pf_coord[2]
                self.glob.ball_coord = [distance*math.cos(course_global_rad)+ self.glob.pf_coord[0],
                                         distance*math.sin(course_global_rad)+ self.glob.pf_coord[1]]
                if len(self.glob.obstacles) == 0: self.glob.obstacles = [[0,0,0]]
                self.glob.obstacles[0] = [self.glob.ball_coord[0], self.glob.ball_coord[1], 0.15]
                return(a, course, distance, 0)
        if with_Localization: self.local.localisation_Complete()
        return False, 0, 0, [0, 0]

    def watch_Ball_In_Pose(self, penalty_Goalkeeper = False):
        self.local.correct_yaw_in_pf()
        if self.robot_In_0_Pose == False:
            if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
                self.kondo.motionPlay(1)
                time.sleep(0.5)
                #self.pyb.delay(500)
            else:
                self.simulateMotion(name = 'Initial_Pose')
            self.robot_In_0_Pose = True
        # U19 - Шея поворот
        # U20 - Шея Наклон
        c = self.neck_play_pose
        head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),
                     (-2667, c-700),(-1333, c-700), (0, c-700), (1333,c-700),(2667, c-700),
                    (-2667, c-1400), (-1333, c-1400), ( 0, c-1400), (1333, c-1400), (2667, c-1400)]
        #head_pose_seq = [2,7,12,13,8,3,1,6,11,10,5,0,4,9,14,2]
        head_pose_seq = [2,7,12,11,6,8,13,14,9,4,3,10,5,0,1,2]
        if penalty_Goalkeeper: head_pose_seq = [2,7,12,11,6,8,13]
        for i in range(len(head_pose_seq)):
            if i != 0:
                x = head_pose[head_pose_seq[i]]
                self.neck_pan = x[0]
                self.neck_tilt = x[1]
            if not self.falling_Test() == 0:
                self.local.quality =0
                if self.falling_Flag == 3: uprint('STOP!')
                else: uprint('FALLING!!!', self.falling_Flag)
                return False, 0, 0, [0, 0]
            if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
                self.kondo.setUserParameter(19,self.neck_pan)
                time.sleep(0.2)
                #self.pyb.delay(200)
                self.kondo.setUserParameter(20,self.neck_tilt)
                time.sleep(0.2)
                #self.pyb.delay(400)
            else:
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , self.neck_pan * self.TIK2RAD * self.FACTOR[21], self.sim.simx_opmode_oneshot)   # Шея поворот
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , self.neck_tilt * self.TIK2RAD * self.FACTOR[22], self.sim.simx_opmode_oneshot)  # Шея Наклон
                for j in range(20):
                    self.sim_simxSynchronousTrigger(self.clientID)
            self.refresh_Orientation()
            a, course, dist, speed = self.detect_Ball_Speed()
            if a == True or (a== False and dist !=0): break
        if a == True or (a== False and dist !=0):
            course_global_rad = course + self.glob.pf_coord[2]
            self.glob.ball_coord = [dist*math.cos(course_global_rad)+ self.glob.pf_coord[0],
                                        dist*math.sin(course_global_rad)+ self.glob.pf_coord[1]]
            if len(self.glob.obstacles) == 0: self.glob.obstacles = [[0,0,0]]
            self.glob.obstacles[0] = [self.glob.ball_coord[0], self.glob.ball_coord[1], 0.15]
            distance = dist *1000
            self.neck_pan =int( - course/ self.TIK2RAD)
            D = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK']- self.params['DIAMETER_OF_BALL']/2
            E = (2*distance*D - math.sqrt(4*distance**2*D**2 - 4*(distance**2-self.params['HEIGHT_OF_NECK']**2)*(D**2 -self.params['HEIGHT_OF_NECK']**2)))/(2*(D**2-self.params['HEIGHT_OF_NECK']**2))
            alpha = math.atan(E)
            alpha_d = math.pi/2 - alpha
            self.neck_tilt = int((-alpha_d)/self.TIK2RAD + self.neck_calibr)
            return(a, course, dist, speed)
        return False, 0, 0, [0, 0]

    def seek_Post_In_Frame(self):
        tra = 0
        for number in range (5):
            if self.glob.SIMULATION == 2:
                #img = self.sensor.snapshot().lens_corr(strength = 1.45, zoom = 1.0)
                ret, img = self.sensor.read()
                blobs = img.find_blobs([self.vision.TH['orange ball']['th']],
                                        pixels_threshold=self.vision.TH['orange ball']['pixel'],
                                        area_threshold=self.vision.TH['orange ball']['area'],
                                        merge=True, margin=10)
                if (len (blobs) == 1):
                    blob = blobs [0]
                    #self.green_led.on()  # подмигивание зеленым светодиодом
                    #time.sleep(50)  #
                    #self.green_led.off() #
                    tra = tra + 1
                    img.draw_rectangle(blob.rect(), color = (255, 0, 0))
                    course, distance = get_course_and_distance_to_post(blob)
                    return True, course, distance, blob
            elif self.glob.SIMULATION == 5:
                import reload as re
                #import cv2
                
                ret, img1 = self.sensor.read()
                import time
                #filename = str(round(time.time(), 2))[:-6] + ".jpg"
                #print(filename)
                #cv2.imwrite(str(round(time.time(), 2))[:-6] + ".jpg", img1)
                img1 = cv2.flip(img1, -1)
                img = re.Image(img1)
               
                
                blobs = img.find_blobs([self.vision.TH['yellow posts']['th']], pixels_threshold=1, area_threshold=1, merge=True)
                if (len (blobs) == 1):
                    blob = blobs [0]
                    tra = tra + 1
                    #img.draw_rectangle(blob.rect())
                    #cv2.imshow("posts", img.img)
                    course, distance = self.get_course_and_distance_to_post(blob)
                    return True, course, distance, blob
            else:
                import reload as re
                #import cv2
                if  self.glob.SIMULATION != 3 :
                    self.sim_simxSynchronousTrigger(self.clientID)
                img1 = self.vision_Sensor_Get_Image()
                img = re.Image(img1)
                self.vision_Sensor_Display(img.img)
                blobs = img.find_blobs([self.vision.TH['yellow posts']['th']], pixels_threshold=1, area_threshold=1, merge=True)
                if (len (blobs) == 1):
                    blob = blobs [0]
                    tra = tra + 1
                    img.draw_rectangle(blob.rect())
                    self.vision_Sensor_Display(img.img)
                    course, distance = self.get_course_and_distance_to_post(blob)
                    return True, course, distance, blob
        if tra == 0: return False, 0, 0, 0

    def seek_Ball_In_Frame(self, with_Localization = True):
        tra = 0
        for number in range (5):
            if self.glob.SIMULATION == 2:
                ret, img = self.sensor.read()
                
                #img = self.sensor.snapshot().lens_corr(strength = 1.45, zoom = 1.0)
                blobs = img.find_blobs([self.vision.TH['orange ball']['th']],
                                        pixels_threshold=self.vision.TH['orange ball']['pixel'],
                                        area_threshold=self.vision.TH['orange ball']['area'],
                                        merge=True, margin=10)
                for blob in blobs:
                    tra = tra + 1
                    img.draw_rectangle(blob.rect(), color = (255, 0, 0))
                    x , y , w , h = blob.rect()  # ball blob
                    x1 = x + w                      # x1, y1, w1, h1-right rectangle
                    y1 = y
                    if x1 + w <= 320:
                        w1 = w
                    else:
                        w1 = 320 - x1
                    if x1 == 320:
                        w1 = 1
                        x1 = 319
                    if y1 + 2 * h <= 240:
                        h1 = 2 * h
                    else:
                        h1 = 240 - y1
                    if y1 + h == 240:
                        h1 = h
                    y2 = y                         # x2, y2, w2, h2 - left rectangle
                    if x - w > 0:
                        x2 = x - w
                        w2 = w
                    else:
                        x2 = 0
                        w2 = x1 - x2
                    if x1 == 0:
                        x2 = 0
                        w2 = 1
                    y2 = y1
                    h2 = h1
                    x3 = x                          # x3, y3, w3, h3 - bottom rectangle
                    y3 = y + h - 1
                    w3 = w
                    h3 = h1 - h + 1
                    blob_p = []                     # right blobs
                    blob_l = []                     # left blobs
                    blob_n = []                     # bottom blobs
                    blob_p = img.find_blobs([self.vision.TH['green field']['th']],roi = [x1 , y1 , w1 , h1], pixels_threshold=7, area_threshold=7, merge=True)
                    blob_l = img.find_blobs([self.vision.TH['green field']['th']],roi = [x2 , y1 , w2 , h1], pixels_threshold=7, area_threshold=7, merge=True)
                    blob_n = img.find_blobs([self.vision.TH['green field']['th']],roi = [x3 , y3 , w3 , h3], pixels_threshold=7, area_threshold=7, merge=True)
                    if len(blob_p) > 0 or len( blob_l ) > 0  or len( blob_n ) > 0: 
                        self.green_led.on()  # подмигивание зеленым светодиодом
                        time.sleep(50)  #
                        self.green_led.off() #
                        break
                    else: tra = 0
                if with_Localization and number == 0: self.local.read_Localization_marks(img)

            elif self.glob.SIMULATION == 5:
                if True:
                #while(True):
                    #print("turn")

                    ret, img1 = self.sensor.read()
                    import time
                    #filename = str(round(time.time(), 2))[:-6] + ".jpg"
                    #print(filename)
                    #cv2.imwrite(str(round(time.time(), 2))[:-6] + ".jpg", img1)
                    img1 = cv2.flip(img1, -1)
                    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2LAB)

                    import reload as re
                    img = re.Image(img1)
                    #img.rotate()
                    
                    if with_Localization and number == 0: self.local.read_Localization_marks(img1)

                 
                blobs, mask = img.find_blobs([self.vision.TH['orange ball']['th']],
                                        pixels_threshold=self.vision.TH['orange ball']['pixel'],
                                        area_threshold=self.vision.TH['orange ball']['area'],
                                        merge=True, return_mask = True)
                
                blobs_boxes = []

                for blob in blobs:
                    r = blob.rect()
                    blobs_boxes.append(((r[0], r[1]), (r[0] + r[2], r[1] + r[3])))

                mask_3_ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

                show_mask_and_annotated_objects(img.img, "ball", mask_3_ch, annotations = blobs_boxes,
                                    annotation_type = "bboxes")
                cv2.waitKey(1) & 0xFF


                #print("len_blobs", len(blobs))
                
                if (len (blobs) == 1):
                    blob = blobs [0]
                    #print("blob ", blob)
                    tra = tra + 1
                    img.draw_rectangle(blob.rect())
                    
            else:
                import reload as re
                if  self.glob.SIMULATION != 3 :
                    self.sim_simxSynchronousTrigger(self.clientID)
                img1 = self.vision_Sensor_Get_Image()
                img = re.Image(img1)
                self.vision_Sensor_Display(img.img)
                if with_Localization and number == 0: self.local.read_Localization_marks(img1)
                blobs = img.find_blobs([self.vision.TH['orange ball']['th']],
                                        pixels_threshold=self.vision.TH['orange ball']['pixel'],
                                        area_threshold=self.vision.TH['orange ball']['area'],
                                        merge=True)
                if (len (blobs) == 1):
                    blob = blobs [0]
                    tra = tra + 1
                    img.draw_rectangle(blob.rect())
                    self.vision_Sensor_Display(img.img)
        if tra == 0: return False, 0, 0, 0
        else:
            #print("blob_to_course, ", blob)
            course, distance = self.get_course_and_distance_to_ball(blob)
            return True, course, distance, blob

    def detect_Ball_Speed(self):
        tra = 0
        position = []
        for number in range (2):
            if self.glob.SIMULATION == 2:
                #img = self.sensor.snapshot().lens_corr(strength = 1.45, zoom = 1.0)
                ret, img1 = self.sensor.read()
                img1 = cv2.flip(img1, -1)

                import reload as re
                img = re.Image(img1)
                
                self.local.read_Localization_marks(img1)
                blobs = img.find_blobs([self.vision.TH['orange ball']['th']],
                                        pixels_threshold=self.vision.TH['orange ball']['pixel'],
                                        area_threshold=self.vision.TH['orange ball']['area'],
                                        merge=True, margin=10)
                if (len (blobs) == 1):
                    blob = blobs [0]
                    tra = tra + 1
                    img.draw_rectangle(blob.rect(), color = (255, 0, 0))
                    x , y , w , h = blob.rect()  # ball blob
                    x1 = x + w                      # x1, y1, w1, h1-right rectangle
                    y1 = y
                    if x1 + w <= 320:
                        w1 = w
                    else:
                        w1 = 320 - x1
                    if x1 == 320:
                        w1 = 1
                        x1 = 319
                    if y1 + 2 * h <= 240:
                        h1 = 2 * h
                    else:
                        h1 = 240 - y1
                    if y1 + h == 240:
                        h1 = h
                    y2 = y                         # x2, y2, w2, h2 - left rectangle
                    if x - w > 0:
                        x2 = x - w
                        w2 = w
                    else:
                        x2 = 0
                        w2 = x1 - x2
                    if x1 == 0:
                        x2 = 0
                        w2 = 1
                    y2 = y1
                    h2 = h1
                    x3 = x                          # x3, y3, w3, h3 - bottom rectangle
                    y3 = y + h - 1
                    w3 = w
                    h3 = h1 - h + 1
                    blob_p = []                     # right blobs
                    blob_l = []                     # left blobs
                    blob_n = []                     # bottom blobs
                    blob_p = img.find_blobs([self.vision.TH['green field']['th']],roi = [x1 , y1 , w1 , h1], pixels_threshold=7, area_threshold=7, merge=True)
                    blob_l = img.find_blobs([self.vision.TH['green field']['th']],roi = [x2 , y1 , w2 , h1], pixels_threshold=7, area_threshold=7, merge=True)
                    blob_n = img.find_blobs([self.vision.TH['green field']['th']],roi = [x3 , y3 , w3 , h3], pixels_threshold=7, area_threshold=7, merge=True)
                    if len(blob_p) > 0 or len( blob_l ) > 0  or len( blob_n ) > 0:
                        #self.green_led.on()  # подмигивание зеленым светодиодом
                        time.sleep(50)  #
                        #self.green_led.off() #
                        course, distance = self.get_course_and_distance_to_ball(blob)
                        position.append([course,distance])
                    else: tra = 0
            elif self.glob.SIMULATION == 5:
                ret, img1 = self.sensor.read()  
                import reload as re
                img1 = cv2.flip(img1, -1)
                img = re.Image(img1)
                
                blobs = img.find_blobs([self.vision.TH['orange ball']['th']],
                                        pixels_threshold = self.vision.TH['orange ball']['pixel'],
                                        area_threshold = self.vision.TH['orange ball']['area'],
                                        merge=True)
                if (len (blobs) == 1):
                    blob = blobs [0]
                    tra = tra + 1
                    img.draw_rectangle(blob.rect())
                    
                    course, distance = self.get_course_and_distance_to_ball(blob)
                    position.append([course,distance])

            else:
                if  self.glob.SIMULATION != 3 :
                    self.sim_simxSynchronousTrigger(self.clientID)
                img1 = self.vision_Sensor_Get_Image()
                img = self.re.Image(img1)
                self.vision_Sensor_Display(img.img)
                blobs = img.find_blobs([self.vision.TH['orange ball']['th']],
                                        pixels_threshold = self.vision.TH['orange ball']['pixel'],
                                        area_threshold = self.vision.TH['orange ball']['area'],
                                        merge=True)
                if (len (blobs) == 1):
                    blob = blobs [0]
                    tra = tra + 1
                    img.draw_rectangle(blob.rect())
                    self.vision_Sensor_Display(img.img)
                    course, distance = self.get_course_and_distance_to_ball(blob)
                    position.append([course,distance])
        n = len(position)
        speed = [0,0]
        if n > 1:
            front_speed = ( position[n-1][1] - position[0][1])/ distance/n
            tangential_speed = ( position[n-1][0] - position[0][0]) * distance/n
            speed = [tangential_speed, front_speed ]
        if tra < 1: return False, 0, 0, [0,0]
        elif tra < 2: return False, course, distance, speed
        else: return True, course, distance, speed


    def get_course_and_distance_to_ball(self, blob):   # returns course in degrees and distance in mm
        c = self.neck_calibr
        if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
            z,a = self.kondo.getUserParameter(19)
            #self.pyb.delay(200)
            time.sleep(0.2)
            z,b = self.kondo.getUserParameter(20)
            a = self.neck_pan
            b = self.neck_tilt
            # U19 - Шея поворот
            # U20 - Шея Наклон
        else:
            returnCode, position21= self.sim.simxGetJointPosition(self.clientID, self.jointHandle[21], self.sim.simx_opmode_blocking)
            a = position21*self.FACTOR[21]*1698        # Шея поворот
            returnCode, position22= self.sim.simxGetJointPosition(self.clientID, self.jointHandle[22], self.sim.simx_opmode_blocking)
            b = position22*self.FACTOR[22]*1698        # Шея Наклон
        x = -(b-c)*0.03375 - (120-blob.cy())* self.params['APERTURE_PER_PIXEL_VERTICAL']     # 0.19406     #
        y1 = math.radians( -(b-c)*0.03375)
        y=math.radians(x)
        vision_height = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK'] + self.params['HEIGHT_OF_NECK']*math.cos(y1)- self.params['DIAMETER_OF_BALL']/2
        distance_in_mm1 = vision_height / math.tan(y) + self.params['HEIGHT_OF_NECK']*math.sin(y1)
        vision_dist = vision_height/math.sin(y)
        vision_shift = vision_dist * math.tan(math.radians((160 - blob.cx()) * self.params['APERTURE_PER_PIXEL']))
        distance_in_mm2 = math.sqrt(distance_in_mm1**2 + vision_shift**2)
        course1 = -a * self.TIK2RAD + math.atan(vision_shift/distance_in_mm1)
        return course1, distance_in_mm2/1000

    def get_course_and_distance_to_post(self, blob_cx, blob_y_plus_h):   # returns course in degrees and distance in mm
        c = self.neck_calibr
        if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
            z,a = self.kondo.getUserParameter(19)
            z,b = self.kondo.getUserParameter(20)
            a = self.neck_pan
            b = self.neck_tilt
            # U19 - Шея поворот
            # U20 - Шея Наклон
        else:
            returnCode, position21= self.sim.simxGetJointPosition(self.clientID, self.jointHandle[21], self.sim.simx_opmode_blocking)
            a = position21*self.FACTOR[21]*1698        # Шея поворот
            returnCode, position22= self.sim.simxGetJointPosition(self.clientID, self.jointHandle[22], self.sim.simx_opmode_blocking)
            b = position22*self.FACTOR[22]*1698        # Шея Наклон
        x = -(b-c)*0.03375 -(120-blob_y_plus_h)* self.params['APERTURE_PER_PIXEL_VERTICAL']
        y1 = math.radians(-(b-c)*0.03375)
        y=math.radians(x)
        vision_height = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK'] + self.params['HEIGHT_OF_NECK']*math.cos(y1)
        if y == 0:
            distance_in_mm1 = vision_dist = 4000
        else:
            distance_in_mm1 = vision_height / math.tan(y) + self.params['HEIGHT_OF_NECK']*math.sin(y1)
            vision_dist = vision_height/math.sin(y)
        vision_shift = vision_dist * math.tan(math.radians((160 - blob_cx) * self.params['APERTURE_PER_PIXEL']))
        distance_in_mm2 = math.sqrt(distance_in_mm1**2 + vision_shift**2)
        course1 = self.euler_angle[0] + math.atan(vision_shift/distance_in_mm1)
        #uprint('distance_in_mm1 = ', distance_in_mm1, 'distance_in_mm2 = ', distance_in_mm2 )
        #uprint('course =', course, 'course1 = ', course1)
        return course1, distance_in_mm2/1000

    def get_cooord_of_point(self, point_x, point_y):   # takes x,y of point in QVGA frame and returns x,y of point in m in local coordinates of robot
        try:
            head_to_horizon_angle = (self.neck_calibr - self.neck_tilt) * 0.03375
            #vision_line_angle = math.radians(head_to_horizon_angle +(point_y - 120) * APERTURE_PER_PIXEL)     # 0.18925
            vision_line_angle = math.radians(head_to_horizon_angle +(point_y - 120) * self.params['APERTURE_PER_PIXEL_VERTICAL'])     # 0.18925
            vision_height = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK'] + self.params['HEIGHT_OF_NECK']*math.cos(math.radians(head_to_horizon_angle))
            distance_in_mm1 = vision_height / math.tan(vision_line_angle) + self.params['HEIGHT_OF_NECK']*math.sin(math.radians(head_to_horizon_angle))
            vision_dist = vision_height/math.sin(vision_line_angle)
            vision_shift = vision_dist * math.tan(math.radians((160 - point_x) * self.params['APERTURE_PER_PIXEL']))
            distance_in_mm2 = math.sqrt(distance_in_mm1**2 + vision_shift**2)
            course =  math.atan(vision_shift/distance_in_mm1) - self.neck_pan * self.TIK2RAD
            x = distance_in_mm2/1000 * math.cos(course)
            y = distance_in_mm2/1000 * math.sin(course)
        except Exception: return False, 0, 0
        return True, x, y

    def turn_To_Course(self, course):
        stepLength = 0
        sideLength = 0
        rotation = 0
        cycleNumber = 1
        cycle = 0
        target = course # + self.direction_To_Attack
        old_neck_pan, old_neck_tilt = self.head_Up()
        self.refresh_Orientation()
        rotation1 = target - self.euler_angle[0]
        if rotation1 > math.pi : rotation1 -= (2 * math.pi)
        if rotation1 < -math.pi : rotation1 += (2 * math.pi)
        if abs(rotation1)> 0.035:
            cycleNumber = int(math.floor(abs(rotation1)/self.params['ROTATION_YIELD']))+1       # rotation yield 0.23 with rotation order 0.21
            self.walk_Initial_Pose()
            for cycle in range (cycleNumber):
                self.refresh_Orientation()
                rotation1 = target - self.euler_angle[0]
                if rotation1 > math.pi : rotation1 -= (2 * math.pi)
                if rotation1 < -math.pi : rotation1 += (2 * math.pi)
                if abs(rotation1)< 0.035: break
                rotation = rotation1*0.23/self.params['ROTATION_YIELD']/(cycleNumber - cycle)
                #uprint('self.euler_angle[0]=', self.euler_angle[0],'rotation =', rotation )
                self.walk_Cycle(stepLength, sideLength,rotation,cycle,cycleNumber)
            self.walk_Final_Pose()
        self.refresh_Orientation()
        self.local.coord_odometry[2] = self.euler_angle[0]
        self.local.coord_shift = [0,0,0]
        self.local.coordinate_record(odometry = True, shift = True)
        self.head_Return(old_neck_pan, old_neck_tilt)

    def head_Up(self):
        old_neck_pan = self.neck_pan
        old_neck_tilt = self.neck_tilt
        self.neck_pan = 0
        self.neck_tilt = self.neck_play_pose
        if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
            self.kondo.setUserParameter(19,0)
            time.sleep(0.2)
            #self.pyb.delay(200)
            self.kondo.setUserParameter(20,self.neck_play_pose)
            time.sleep(0.4)
            #self.pyb.delay(400)
        else:
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[21] , 0, self.sim.simx_opmode_oneshot)   # Шея поворот
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[22] , self.neck_play_pose*0.000589*self.FACTOR[22], self.sim.simx_opmode_oneshot)  # Шея Наклон
            for i in range(16):
                self.sim_simxSynchronousTrigger(self.clientID)
        self.refresh_Orientation()
        return old_neck_pan, old_neck_tilt

    def head_Return(self, old_neck_pan, old_neck_tilt):
        if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
            self.kondo.setUserParameter(19,old_neck_pan)
            time.sleep(0.2)
            #self.pyb.delay(200)
            self.kondo.setUserParameter(20,old_neck_tilt)
            time.sleep(0.4)
            #self.pyb.delay(400)
        else:
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[21] , old_neck_pan*0.000589*self.FACTOR[21], self.sim.simx_opmode_oneshot)   # Шея поворот
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[22] , old_neck_tilt*0.000589*self.FACTOR[22], self.sim.simx_opmode_oneshot)  # Шея Наклон
            for i in range(16):
                self.sim_simxSynchronousTrigger(self.clientID)
        self.refresh_Orientation()

    def localisation_Motion(self):
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
            self.kondo.motionPlay(1)
            #self.pyb.delay(500)
            time.sleep(0.5)
        # U19 - Шея поворот
        # U20 - Шея Наклон
        c = self.neck_play_pose
        head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),
                     (-2667, c-700),(-1333, c-700), (0, c-700), (1333,c-700),(2667, c-700),
                    (-2667, c-1400), (-1333, c-1400), ( 0, c-1400), (1333, c-1400), (2667, c-1400)]
        #head_pose_seq = [2,7,12,11,6,8,13,14,9,4,3,10,5,0,1,2]
        head_pose_seq = [2,7,6,8,9,4,3,5,0,1,2]
        for k in range(1):
            for i in range(len(head_pose_seq)):
                x = head_pose[head_pose_seq[i]]
                self.neck_pan = x[0]
                self.neck_tilt = x[1]
                if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
                    self.kondo.setUserParameter(19,self.neck_pan)
                    time.sleep(0.2)
                    #self.pyb.delay(200)
                    self.kondo.setUserParameter(20,self.neck_tilt)
                    time.sleep(0.2)
                    #self.pyb.delay(200)
                else:
                    returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                             self.jointHandle[21] , self.neck_pan*0.000589*self.FACTOR[21], self.sim.simx_opmode_oneshot)   # Шея поворот
                    returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                             self.jointHandle[22] , self.neck_tilt*0.000589*self.FACTOR[22], self.sim.simx_opmode_oneshot)  # Шея Наклон
                    for i in range(16):
                        self.sim_simxSynchronousTrigger(self.clientID)
                self.refresh_Orientation()
                a, course, distance, blob = self.seek_Ball_In_Frame()
            #self.local.pf_update()
            #a = self.local.process_Post_data_in_Pose()
            #if self.local.quality == 1 : break
            #target_course1 = self.euler_angle[0] +180
            #self.turn_To_Course(target_course1)
        a = self.local.localisation_Complete()
        #self.local.pf_update()
        return a

    def normalize_rotation(self, yaw):
        if abs(yaw) > 2 * math.pi: yaw %= (2 * math.pi)
        if yaw > math.pi : yaw -= (2 * math.pi)
        if yaw < -math.pi : yaw += (2 * math.pi)
        if yaw > 0.5 : yaw = 0.5
        if yaw < -0.5 : yaw = -0.5
        return yaw

    def near_distance_omni_motion(self, dist_mm, napravl):
        dist = dist_mm/1000
        self.refresh_Orientation()
        initial_direction = self.imu_body_yaw()
        n = int(math.floor((dist_mm*math.cos(napravl)-self.first_step_yield)/self.cycle_step_yield)+1)+1         #calculating the number of potential full steps forward
        displacement = dist_mm*math.sin(napravl)
        if displacement > 0:
            invert = -1
            self.first_Leg_Is_Right_Leg = False
            side_step_yield = self.side_step_left_yield
        else:
            invert = 1
            side_step_yield = self.side_step_right_yield
        m = int(math.floor(abs(displacement)/side_step_yield)+1)
        if n < m : n = m
        stepLength = dist_mm*math.cos(napravl)/(self.first_step_yield*1.25+self.cycle_step_yield*(n-1)+ self.cycle_step_yield*0.75)*64
        number_Of_Cycles = n+2
        sideLength = abs(displacement) /number_Of_Cycles*20/side_step_yield
        if stepLength > 15 and number_Of_Cycles > 4: 
            deceleration = True
            number_Of_Cycles += 1
        else: deceleration = False
        old_neck_pan, old_neck_tilt = self.head_Up()
        self.local.correct_yaw_in_pf()
        self.walk_Initial_Pose()
        for cycle in range(number_Of_Cycles):
            self.refresh_Orientation()
            rotation = initial_direction - self.imu_body_yaw() * 1
            rotation = self.normalize_rotation(rotation)
            stepLength1 = stepLength
            if cycle == 0: stepLength1 = stepLength/4
            if cycle == 1: stepLength1 = stepLength/2
            if deceleration:
                if cycle == number_Of_Cycles - 1: stepLength1 = stepLength / 3
                if cycle == number_Of_Cycles - 2: stepLength1 = stepLength * 2 / 3
            self.walk_Cycle(stepLength1, sideLength, invert*rotation,cycle,number_Of_Cycles)
        self.walk_Final_Pose()
        self.first_Leg_Is_Right_Leg = True
        self.local.coord_odometry[0] += dist * math.cos(napravl)
        self.local.coord_odometry[1] += dist * math.sin(napravl)
        #self.local.coordinate_record(odometry = True)
        self.head_Return(old_neck_pan, old_neck_tilt)

    def near_distance_ball_approach_and_kick(self, kick_direction, strong_kick = False, small_kick = False ):
        offset_of_ball = self.params['KICK_OFFSET_OF_BALL']  # self.d10 # module of local robot Y coordinate of ball im mm before kick 
        a, napravl, dist, speed = self.seek_Ball_In_Pose(fast_Reaction_On = True)
        dist_mm = dist *1000
        if a==False or self.falling_Flag != 0: return False
        if dist > 0.9 or a == False: return False
        if  0.02 < abs(dist * math.cos(napravl)) < 0.06 and dist * math.sin(napravl) < 0.03:
            old_neck_pan, old_neck_tilt = self.head_Up()
            if napravl > 0: self.kick(first_Leg_Is_Right_Leg=False)
            else: self.kick(first_Leg_Is_Right_Leg=True)
            self.head_Return(old_neck_pan, old_neck_tilt)
        if abs(napravl) > 1 :
            direction = math.copysign(2.55, napravl)
            self.near_distance_omni_motion( 180 , direction)
        else:
            forth_dist = dist_mm*math.cos(napravl) 
            n = int(math.ceil((forth_dist - self.params['KICK_ADJUSTMENT_DISTANCE']
                                -self.first_step_yield)/self.cycle_step_yield)+1)         #calculating the number of potential full steps forward
            displacement = dist_mm*math.sin(napravl)- math.copysign(offset_of_ball, napravl)
            if displacement > 0:
                invert = -1
                self.first_Leg_Is_Right_Leg = False
                side_step_yield = self.side_step_left_yield
            else:
                invert = 1
                side_step_yield = self.side_step_right_yield
            m = int(math.ceil(abs(displacement)/side_step_yield))
            if n < m : n = m
            n += 2
            stepLength = (dist_mm*math.cos(napravl)-
                          self.params['KICK_ADJUSTMENT_DISTANCE'])/(self.first_step_yield
                          + self.cycle_step_yield * n) * 64
            number_Of_Cycles = n + 2
            if napravl > 0:
                kick_by_Right = False
            else:
                kick_by_Right = True
            sideLength = abs(displacement)/number_Of_Cycles*20/side_step_yield
            old_neck_pan, old_neck_tilt = self.head_Up()
            self.local.correct_yaw_in_pf()
            kick_direction = self.imu_body_yaw()
            init_yaw = self.imu_body_yaw()
            stepLengthResidue = 0
            sideLengthResidue = 0
            self.walk_Initial_Pose()
            cycle = 0
            while (cycle < number_Of_Cycles):
            #for cycle in range(number_Of_Cycles):
                self.refresh_Orientation()
                rotation = (kick_direction - self.imu_body_yaw()) * 1
                rotation = self.normalize_rotation(rotation)
                stepLength1 = stepLength
                #print('kick_direction =', kick_direction,'self.imu_body_yaw() = ', self.imu_body_yaw(), 'rotation = ', rotation )
                if cycle == 0: stepLength1 = stepLength / 3
                if cycle == 1: stepLength1 = stepLength * 2 / 3
                stepLength1 += stepLengthResidue
                sideLength += sideLengthResidue
                print('sideLength = ', sideLength)
                #if abs(sideLength) > 20:
                #    sideLength = sideLength / abs(sideLength) * 20
                self.walk_Cycle(stepLength1, sideLength, invert*rotation,cycle,number_Of_Cycles)
                delta_yaw = self.norm_yaw(self.imu_body_yaw() - init_yaw)
                stepLengthResidue = stepLength1 * (1 - math.cos(delta_yaw)) - sideLength * math.sin(delta_yaw) * invert
                sideLengthResidue = sideLength * (1 - math.cos(delta_yaw)) + stepLength1 * math.sin(delta_yaw) * invert
                cycle += 1
            self.walk_Final_Pose()
            self.first_Leg_Is_Right_Leg = True
            if strong_kick == True:
                if kick_by_Right == True:
                    self.play_Motion_Slot(name = 'Soccer_Kick_Forward_Right_Leg')
                else:
                    self.play_Motion_Slot(name = 'Soccer_Kick_Forward_Left_Leg')
            else:
                self.kick( first_Leg_Is_Right_Leg=kick_by_Right, small = small_kick)
            self.local.coord_odometry[0] += dist * math.cos(napravl)
            self.local.coord_odometry[1] += dist * math.sin(napravl)
            #self.local.coordinate_record(odometry = True)
            self.head_Return(old_neck_pan, old_neck_tilt)
        return True

    def far_distance_ball_approach(self, ball_coord):
        old_neck_pan, old_neck_tilt = self.head_Up()
        self.local.correct_yaw_in_pf()
        ball_Approach(self, self.local, self.glob, ball_coord)
        self.head_Return(old_neck_pan, old_neck_tilt)

    def far_distance_plan_approach(self, ball_coord, target_yaw, stop_Over = False):
        target_x = ball_coord[0] - 0.20 * math.cos(target_yaw)
        target_y = ball_coord[1] - 0.20 * math.sin(target_yaw)
        target_coord = [target_x, target_y, target_yaw]
        dest, centers, price = self.p.path_calc_optimum(self.glob.pf_coord, target_coord)
        print("Calc path, ", dest, centers, price)
        #if price > 100:
        #    self.far_distance_ball_approach(ball_coord)
        #    return True
        if stop_Over: price += 100
        #print('centers:', centers)
        start_yaw = self.glob.pf_coord[2]  #self.imu_body_yaw()
        if len(dest) == 0: return False
        old_neck_pan, old_neck_tilt = self.head_Up()
        self.local.correct_yaw_in_pf()
        sideLength = 0
        stepLength_old = 0
        acceleration = False
        deceleration = False
        self.walk_Initial_Pose()
# initial arc
        dest_yaw = self.p.coord2yaw(dest[1][0] - dest[0][0], dest[1][1] - dest[0][1] )
        x1, y1, x2, y2, cx, cy, R, CW = centers[0]
        delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
        number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
        delta_yaw_step = delta_yaw / number_Of_Cycles
        stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
        if stepLength - stepLength_old > 22 :
            acceleration = True
            number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if acceleration:
                if cycle == 0: stepLength1 = stepLength / 3
                if cycle == 1: stepLength1 = stepLength * 2 / 3
            self.refresh_Orientation()
            rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
            rotation = self.normalize_rotation(rotation)
            self.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles+1)
        stepLength_old = stepLength
        acceleration = False
# 1-st straight segment 
        L = math.sqrt((dest[1][0] - dest[0][0])**2 + (dest[1][1] - dest[0][1])**2)
        number_Of_Cycles = math.ceil(abs(L * 1000 / self.cycle_step_yield))
        stepLength = L * 1000 / number_Of_Cycles * 64 / self.cycle_step_yield
        if stepLength - stepLength_old > 22 :
            acceleration = True
            number_Of_Cycles += 1
        if not stop_Over:
            for cycle in range(number_Of_Cycles):
                stepLength1 = stepLength
                if acceleration:
                    if cycle == 0: stepLength1 = stepLength / 3
                    if cycle == 1: stepLength1 = stepLength * 2 / 3
                self.refresh_Orientation()
                rotation = dest_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles+2)
            stepLength_old = stepLength
            acceleration = False
            for i in range(len(centers)-2):
                start_yaw = dest_yaw   #self.imu_body_yaw()
                dest_yaw = self.p.coord2yaw(dest[2*i+3][0] - dest[2*i+2][0], dest[2*i+3][1] - dest[2*i+2][1])
                x1, y1, x2, y2, cx, cy, R, CW = centers[i+1]
                delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
                number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
                delta_yaw_step = delta_yaw / number_Of_Cycles
                stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
                if stepLength - stepLength_old > 22 :
                    acceleration = True
                    number_Of_Cycles += 1
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    if acceleration:
                        if cycle == 0: stepLength1 = stepLength / 3
                        if cycle == 1: stepLength1 = stepLength * 2 / 3
                    self.refresh_Orientation()
                    rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
                    rotation = self.normalize_rotation(rotation)
                    if price < 100:
                        self.walk_Cycle(stepLength1, sideLength, rotation, cycle+1, number_Of_Cycles+2)
                    else:
                        self.walk_Cycle(stepLength1, sideLength, rotation, cycle+1, number_Of_Cycles+1)
                stepLength_old = stepLength
                acceleration = False
                if price >= 100: break
                L = math.sqrt((dest[2*i+3][0] - dest[2*i+2][0])**2 + (dest[2*i+3][1] - dest[2*i+2][1])**2)
                number_Of_Cycles = math.ceil(abs(L * 1000 / self.cycle_step_yield))
                stepLength = L * 1000 / number_Of_Cycles * 64 / self.cycle_step_yield
                if stepLength - stepLength_old > 22 :
                    acceleration = True
                    number_Of_Cycles += 1
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    if acceleration:
                        if cycle == 0: stepLength1 = stepLength / 3
                        if cycle == 1: stepLength1 = stepLength * 2 / 3
                    self.refresh_Orientation()
                    rotation = dest_yaw - self.imu_body_yaw()
                    rotation = self.normalize_rotation(rotation)
                    self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles+2)
                stepLength_old = stepLength
                acceleration = False
            if price < 100:
                start_yaw = dest_yaw   #self.imu_body_yaw()
                dest_yaw = target_yaw
                x1, y1, x2, y2, cx, cy, R, CW = centers[len(centers)-1]
                delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
                number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
                delta_yaw_step = delta_yaw / number_Of_Cycles
                stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
                if stepLength - stepLength_old > 22 :
                    acceleration = True
                    number_Of_Cycles += 1
                if stepLength > 15:
                    deceleration = True
                    number_Of_Cycles += 1
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    if acceleration:
                        if cycle == 0: stepLength1 = stepLength / 3
                        if cycle == 1: stepLength1 = stepLength * 2 / 3
                    if deceleration:
                        if cycle == number_Of_Cycles - 1: stepLength1 = stepLength / 3
                        if cycle == number_Of_Cycles - 2: stepLength1 = stepLength * 2 / 3
                    self.refresh_Orientation()
                    rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
                    rotation = self.normalize_rotation(rotation)
                    self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles + 1)
            # Adjustment of yaw position
            number_Of_Cycles = 4
            stepLength = 0
            for cycle in range(1, number_Of_Cycles+1, 1):
                self.refresh_Orientation()
                rotation = target_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                self.walk_Cycle(stepLength, sideLength, rotation, cycle, number_Of_Cycles+1)
        else:
            number_Of_Cycles = math.ceil(number_Of_Cycles/2)
            if stepLength > 15:
                    deceleration = True
                    number_Of_Cycles += 1
            else: deceleration = False
            for cycle in range(1, number_Of_Cycles + 1, 1):
                stepLength1 = stepLength
                if acceleration:
                    if cycle == 0: stepLength1 = stepLength / 3
                    if cycle == 1: stepLength1 = stepLength * 2 / 3
                self.refresh_Orientation()
                rotation = dest_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                if deceleration:
                    if cycle == number_Of_Cycles: stepLength1 = stepLength / 3
                    if cycle == number_Of_Cycles - 1: stepLength1 = stepLength * 2 / 3
                self.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles + 1)
        self.walk_Final_Pose()
        self.head_Return(old_neck_pan, old_neck_tilt)
        return True


if __name__=="__main__":
    print('This is not main module!')


