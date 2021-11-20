#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json

#from ball_Approach_Steps_Seq import *
from compute_Alpha_v3 import Alpha
from path_planning import PathPlan



def uprint(*text):
    #with open(current_work_directory + "Soccer/log/output.txt",'a') as f:
    #    print(*text, file = f)
    print(*text )

class Motion1:

    def __init__(self, glob, vision, button):
        self.glob = glob
        self.button = button
        self.params = self.glob.params
        self.ACTIVESERVOS = [(10,2),(9,2),(8,2),(7,2),(6,2),(5,2),(4,2),
                (3,2),(2,2),(1,2),(0,2),(10,1),(9,1),(8,1),
                (7,1),(6,1),(5,1),(4,1),(3,1),(2,1),(1,1)]
        # (10,2) Прав Стопа Вбок, (9,2) Прав Стопа Вперед,(8,2) Прав Голень, (7,2) Прав Колено,
        # (6,2)  Прав Бедро,      (5,2) Прав Таз,         (1,2) Прав Ключица,(4,2) Прав Локоть, (0,2) Торс
        # (10,1) Лев Стопа Вбок,  (9,1) Лев Стопа Вперед, (8,1) Лев Голень,  (7,1) Лев Колено
        # (6,1)  Лев Бедро,       (5,1) Лев Таз,          (1,1) Лев Ключица, (4,1) Лев Локоть

        #FACTOR =  [ 1,-1,-1,1,-1,-1, 1,1,1,-1,1,-1,-1, 1,1,-1,-1, 1,1,1,-1,-1, 1]  # v2.3
        self.FACTOR =  [ 1,1,1,-1,1,1, 1,1,1,1,1,1,1, 1,-1,1,1, 1,1,1,1, 1, 1]  # Surrogat 1
        a5 = 21.5  # мм расстояние от оси симметрии до оси сервы 5
        b5 = 18.5  # мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
        c5 = 0     # мм расстояние от оси сервы 6 до нуля Z по вертикали
        a6 = 42    # мм расстояние от оси сервы 6 до оси сервы 7
        a7 = 65.5  # мм расстояние от оси сервы 7 до оси сервы 8
        a8 = 63.8  # мм расстояние от оси сервы 8 до оси сервы 9
        a9 = 35.5  # мм расстояние от оси сервы 9 до оси сервы 10
        a10= 25.4  # мм расстояние от оси сервы 10 до центра стопы по горизонтали
        b10= 16.4  # мм расстояние от оси сервы 10 до низа стопы
        c10 = 12   # мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
        self.SIZES = [ a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 ]
        self.d10 = 53.4 #53.4 # расстояние по Y от центра стопы до оси робота
        limAlpha5 = [-2667, 2667]
        limAlpha6 = [-3000,  740]
        limAlpha7 = [-3555, 3260]
        limAlpha8 = [-4150, 1777]
        limAlpha9 = [-4000, 2960]
        limAlpha10 =[-2815,   600]
        LIMALPHA = [limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10]
        self.MOTION_SLOT_DICT = {0:['',0], 1:['',0], 2:['',0], 3:['',0], 4:['',0], 5:['Get_Up_Inentification',7000],
                    6:['Soccer_Get_UP_Stomach_N', 5000], 7:['Soccer_Get_UP_Face_Up_N', 5000], 8:['Soccer_Walk_FF',0], 9:['',0], 10:['',0],
                    11:['',0], 12:['',0], 13:['',0], 14:['Soccer_Small_Jump_Forward',0], 15:['',0],
                    16:['',0], 17:['',0], 18:['Soccer_Kick_Forward_Right_Leg',5000], 19: ['Soccer_Kick_Forward_Left_Leg',5000], 20:['',0],
                    21:['Get_Up_From_Defence',1000], 22:['',0], 23:['PanaltyDefenceReady_Fast',500], 24:['PenaltyDefenceF',300], 25:['',0],
                    26:['Soccer_Speed_UP',0], 27:['',0], 28:['',0], 29:['',0], 30:['Soccer_Walk_FF0',0],
                    31:['Soccer_Walk_FF1',0], 32:['Soccer_Walk_FF2',0], 33: ['Soccer_Get_UP_Stomach',0], 34:['Soccer_Get_UP_Face_Up',0],
                    35: ['Get_Up_Right',0], 36: ['PenaltyDefenceR',2000], 37: ['PenaltyDefenceL',2000]}
        self.TIK2RAD = 0.00058909
        self.slowTime   = 0.0             # seconds
        self.simThreadCycleInMs = 20
        self.frame_delay = self.glob.params['FRAME_DELAY']
        self.frames_per_cycle = self.glob.params['FRAMES_PER_CYCLE']
        self.motion_shift_correction_x = -self.glob.params['MOTION_SHIFT_TEST_X'] / 21
        self.motion_shift_correction_y = -self.glob.params['MOTION_SHIFT_TEST_Y'] / 21
        self.first_step_yield = self.glob.first_step_yield
        self.cycle_step_yield = self.glob.cycle_step_yield
        self.side_step_right_yield = self.glob.side_step_right_yield
        self.side_step_left_yield = self.glob.side_step_left_yield
        self.stepLength = 0.0    # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
        self.sideLength = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
        self.rotation = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.
        self.first_Leg_Is_Right_Leg = True
        # Following paramenetrs Not recommended for change
        self.amplitude = 32          # mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
        self.fr1 =8                  # frame number for 1-st phase of gait ( two legs on floor)
        self.fr2 = 12                # frame number for 2-nd phase of gait ( one leg in air)
        self.gaitHeight= 180         # Distance between Center of mass and floor in walk pose
        self.stepHeight = 32.0       # elevation of sole over floor
        self.initPoses = 400//self.simThreadCycleInMs
        self.limAlpha1 =LIMALPHA
        self.limAlpha1[3][1]=0
        #  end of  paramenetrs Not recommended for change
        self.al = Alpha()
        self.exitFlag = 0
        self.falling_Flag = 0
        self.neck_pan = 0
        self.old_neck_pan = 0
        self.body_euler_angle =[]
        self.local = 0 # Local
        self.vision = vision
        self.p = PathPlan(self.glob)
        self.old_neck_tilt = 0
        self.direction_To_Attack = 0
        self.activePose = []
        self.xtr = 0
        self.ytr = -self.d10   #-53.4
        self.ztr = -self.gaitHeight
        self.xr = 0
        self.yr = 0
        self.zr = -1
        self.wr = 0
        self.xtl = 0
        self.ytl = self.d10   # 53.4
        self.ztl = -self.gaitHeight
        self.xl = 0
        self.yl = 0
        self.zl = -1
        self.wl = 0
        self.euler_angle = []
        self.robot_In_0_Pose = False
        self.ACTIVEJOINTS = ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5','hand_right_4',
            'hand_right_3','hand_right_2','hand_right_1','Tors1','Leg_left_10','Leg_left_9','Leg_left_8',
            'Leg_left_7','Leg_left_6','Leg_left_5','hand_left_4','hand_left_3','hand_left_2','hand_left_1','head0','head12']
        if self.glob.SIMULATION == 2 or 5:
            #from button_test import wait_for_button_pressing
            #import starkit
            #import pyb
            from kondo_controller import Rcb4BaseLib
            #from pyb import Pin, UART, LED
            #import sensor as s
            #from machine import I2C
            #from bno055 import BNO055, AXIS_P7
            #self.wait_for_button_pressing = wait_for_button_pressing
            #self.starkit = starkit
            #i2c = I2C(2)
            #self.imu = BNO055(i2c, mode = 0x08)
            self.imu = self.button
            self.imu.init_imu()
            self.imu.euler()  
            #self.green_led = LED(2)
            #self.pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)
            #uart = UART(self.glob.params['UART_PORT'], self.glob.params['UART_SPEED'], timeout=1000, parity=0)
            #uart = pyb.UART(1, 1250000, parity=0)
            #k = kondo.Kondo()
            ##k.init(uart)

            self.kondo = Rcb4BaseLib()
            self.kondo.open('/dev/ttyS5', 1250000, 1.3)#1250000, 1.3)
            #115200 1250000
            #self.kondo = Rcb4BaseLib()
            #self.kondo.open(uart)
            #self.clock = time.clock()

            # s.reset()
            # s.set_pixformat(s.RGB565)
            # s.set_framesize(s.QVGA)
            # s.skip_frames(time = 500)
            # s.set_auto_exposure(False)
            # s.set_auto_whitebal(False)
            # s.skip_frames(time = 500)
            #s.set_auto_gain(False, gain_db=0 ) # 9.172951)
            #s.skip_frames(time = 500)
            #s.set_auto_exposure(False, exposure_us=2000) #6576)
            #s.skip_frames(time = 500)
            #s.set_auto_whitebal(False, rgb_gain_db = (-6.0, -3.0, 3.0))
            #self.sensor = s
            self.kondo.motionPlay(25)
            #self.pyb = pyb
            with open(self.glob.current_work_directory + "Init_params/Real/Real_calibr.json", "r") as f:
                data1 = json.loads(f.read())
            self.neck_calibr = data1['neck_calibr']
            self.neck_play_pose = data1['neck_play_pose']
            self.neck_tilt = self.neck_calibr

    #-------------------------------------------------------------------------------------------------------------------------------
    def imu_body_yaw(self):
        yaw = self.neck_pan*self.TIK2RAD + self.euler_angle[0]
        yaw = self.norm_yaw(yaw)
        return yaw

    def norm_yaw(self, yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:  yaw -= 2* math.pi
        if yaw < -math.pi: yaw += 2* math.pi
        return yaw

    def quaternion_to_euler_angle(self, quaternion):
        w,x,y,z = quaternion
        ysqr = y*y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0,t1))
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3,t4))

        return X, Y, Z

    def push_Button(self, button):
        pressed_button = button.get_button()
        uprint("нажато")
        return pressed_button
        #ala = 0
        #while(ala==0):
        #    if (self.pin2.value()== 0):   # нажатие на кнопку 2 на голове
        #        ala = 1
        #        uprint("нажато")
        #self.pyb.delay(1000)

    def play_Motion_Slot(self, name = ''):
        if self.glob.SIMULATION == 2 or 5:
            for key in self.MOTION_SLOT_DICT:
                if self.MOTION_SLOT_DICT[key][0] == name:
                    self.kondo.motionPlay(key)
                    time.sleep((self.MOTION_SLOT_DICT[key][1])/1000)
                    #self.pyb.delay(self.MOTION_SLOT_DICT[key][1])
        else:
            self.simulateMotion(name = name)

    def play_Soft_Motion_Slot(self, name = ''):             # the slot from file will be played in robot 
        if self.glob.SIMULATION == 2 or 5:
            with open(self.glob.current_work_directory + "Soccer/Motion/motion_slots/" + name + ".json", "r") as f:
                slots = json.loads(f.read())
            motion_list = slots[name]
            for motion in motion_list:
                servoDatas = []
                for i in range(len(self.ACTIVESERVOS)):
                    pos = int(motion[i+1]) + 7500 #- self.servo_Trims[i]
                    servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                servoDatas = self.reOrderServoData(servoDatas)
                frames_number = int(motion[0]) 
                a=self.kondo.setServoPos (servoDatas, frames_number)
                time.sleep((30 * frames_number)/1000)
                #self.pyb.delay(30 * frames_number)
                #self.pyb.delay(250 )
        else:
            self.simulateMotion(name = name)

    def falling_Test(self):
        if self.glob.SIMULATION == 0 or self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
            key = 0
            if self.ms.kbhit():
                key = self.ms.getch()
            if key == b'p' :
                self.lock.acquire()
                if self.glob.SIMULATION == 3:
                    self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                key = 0
                while (True):
                    if self.ms.kbhit():
                        key = self.ms.getch()
                    if key == b'p':
                        self.lock.release()
                        if self.glob.SIMULATION == 3:
                            self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                        key = 0
                        break
            if key == b's' or self.transfer_Data.stop_Flag:
                uprint('Simulation STOP by keyboard')
                self.transfer_Data.stop += 1
                self.sim_Stop()
                while True:
                    if self.transfer_Data.stop == self.numberOfRobots:
                        self.sim_Disable()
                        sys.exit(0)
                    time.sleep(0.1)
                self.falling_Flag = 3
                self.transfer_Data.stop_Flag = True
                return self.falling_Flag
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
            self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
            if (self.body_euler_angle[1]) < -45:
                self.falling_Flag = 1                   # on stomach
                self.simulateMotion(name = 'Soccer_Get_UP_Stomach_N')
            if (self.body_euler_angle[1]) >  45:
                self.falling_Flag = -1                  # face up
                self.simulateMotion(name = 'Soccer_Get_UP_Face_Up')
            if 45< (self.body_euler_angle[2]) < 135:
                self.falling_Flag = -2                  # on right side
                self.simulateMotion(name = 'Get_Up_Right')
            if -135< (self.body_euler_angle[2]) < -45:
                self.falling_Flag = 2                   # on left side
                self.simulateMotion(name = 'Get_Up_Left')
        if self.glob.SIMULATION == 2 or 5:
            #ad = self.kondo.getAllAdData()
            #uprint(ad)
            #if ad[0] == True:
            #ad3 = ad[1][3]
            #ad4 = ad[1][4]
            ad3 = self.kondo.getAdData(3)
            ad4 = self.kondo.getAdData(4)
            if ad3 < 200:
                self.falling_Flag = 1     # on stomach
                #self.kondo.motionPlay(6)
                #self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                #self.pyb.delay(6000)
                self.play_Soft_Motion_Slot(name = 'Soccer_Get_UP_Stomach_N')
            if ad3 > 450:
                self.falling_Flag = -1    # face up
                self.kondo.motionPlay(7)
                self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                time.sleep(6)
                #self.pyb.delay(6000)
            if ad4 > 400:
                self.falling_Flag = -2    # on right side
                self.kondo.motionPlay(5)
                self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                time.sleep(8)
                #self.pyb.delay(8000)
            if ad4 < 160:
                self.falling_Flag = 2     # on left side
                self.kondo.motionPlay(5)
                self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                time.sleep(8)
                #self.pyb.delay(8000)

        self.falling_Flag = 0
        return self.falling_Flag
        

    def computeAlphaForWalk(self,sizes, limAlpha, hands_on = True ):
        angles =[]
        anglesR=[]
        anglesL=[]
        
        anglesR = self.al.compute_Alpha_v3(self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, sizes, limAlpha)
        anglesL = self.al.compute_Alpha_v3(self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,self.wl, sizes, limAlpha)

        if len(anglesR)>1:
            for i in range(len(anglesR)):
                if len(anglesR)==1: break
                if anglesR[0][2]<anglesR[1][2]: anglesR.pop(1)
                else: anglesR.pop(0)
        elif len(anglesR)==0:
            return[]
        if len(anglesL)>1:
            for i in range(len(anglesL)):
                if len(anglesL)==1: break
                if anglesL[0][2]<anglesL[1][2]: anglesL.pop(1)
                else: anglesL.pop(0)
        elif len(anglesL)==0:
            return[]
        if self.first_Leg_Is_Right_Leg == True:
            for j in range(6): angles.append(anglesR[0][j])
            if hands_on: angles.append(1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.524 - self.xtl/57.3)
            else: angles.append(0.0)
            angles.append(0.0)
            #for j in range(5): angles.append(0.0)
            for j in range(6): angles.append(-anglesL[0][j])
            #for j in range(4): angles.append(0.0)
            if hands_on: angles.append(-1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.524 + self.xtr/57.3)
            else: angles.append(0.0)
        else:
            for j in range(6): angles.append(anglesL[0][j])
            if hands_on: angles.append(1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.524 - self.xtr/57.3)
            else: angles.append(0.0)
            angles.append(0.0)                                  # Tors
            for j in range(6): angles.append(-anglesR[0][j])
            if hands_on: angles.append(-1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.524 + self.xtl/57.3)
            else: angles.append(0.0)
        self.activePose = angles
        return angles

    def activation(self):
        if self.glob.SIMULATION == 2 or 5:
            t1t1 = time.time()
            yaw, roll, pitch = self.imu.euler()
            print('!!!!!', time.time() - t1t1)
            if 0<= yaw < 180: yaw = -yaw
            if 180<= yaw <=360: yaw = 360 -yaw
            self.euler_angle = [yaw, roll, pitch]
            self.euler_angle[0] = math.radians(self.euler_angle[0])
            self.direction_To_Attack += self.euler_angle[0]
        else:
            time.sleep(0.1)
            if self.glob.SIMULATION != 0:
                self.sim.simxStartSimulation(self.clientID,self.sim.simx_opmode_oneshot)
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
            self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
            returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
            self.euler_angle = list(self.quaternion_to_euler_angle(Dummy_Hquaternion))
            self.euler_angle[0] = math.radians(self.euler_angle[0])
            self.direction_To_Attack += self.euler_angle[0]
            #uprint('body_euler_angle = ', self.body_euler_angle)
            returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
            self.Dummy_HData.append(Dummy_Hposition)
            returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        self.direction_To_Attack = self.norm_yaw(self.direction_To_Attack)

    def reOrderServoData(self, servoDatas):
        order = [0, 11, 1, 12, 2, 13, 3, 14, 4, 15, 5, 16, 6, 17, 7, 18, 8, 19, 9, 20, 10]
        servoDatasOrdered = []
        for orderNumber in order:
            servoDatasOrdered.append(servoDatas[orderNumber])
        return servoDatasOrdered

    def walk_Initial_Pose(self):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        self.xtr = self.xtl = 0
        framestep = self.simThreadCycleInMs//10
        for j in range (self.initPoses):
            if self.glob.SIMULATION == 2 or 5: start1 = time.time()*1000
            self.ztr = -223.0 + j*(223.0-self.gaitHeight)/self.initPoses
            self.ztl = -223.0 + j*(223.0-self.gaitHeight)/self.initPoses
            self.ytr = -self.d10 - j*self.amplitude/2 /self.initPoses
            self.ytl =  self.d10 - j*self.amplitude/2 /self.initPoses
            angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                            returnCode = self.sim.simxSetJointPosition(self.clientID,
                                         self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                         self.sim.simx_opmode_oneshot)
                    self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        #self.Dummy_HData.append(Dummy_Hposition)
                        #self.timeElapsed = self.timeElapsed +1
                        # uprint(Dummy_Hposition)
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                            self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                    if self.glob.SIMULATION == 1:
                        self.sim_simxSynchronousTrigger(self.clientID)
                elif self.glob.SIMULATION == 2 or 5:
                    servoDatas = []
                    for i in range(len(angles)):
                        pos = int(angles[i]*1698 + 7500)
                        servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                    servoDatas = self.reOrderServoData(servoDatas)
                    #start2 = self.pyb.millis()
                    a=self.kondo.setServoPos (servoDatas, self.frames_per_cycle)
                    #uprint(servoDatas)
                    #uprint(clock.avg())
                    #time1 = self.pyb.elapsed_millis(start1)
                    time1 = time.time()*1000 - start1
                    #time2 = self.pyb.elapsed_millis(start2)
                    time.sleep((self.frame_delay - time1)/1000)
                    #self.pyb.delay(self.frame_delay - time1)

    def walk_Cycle(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        self.stepLength = stepLength + self.motion_shift_correction_x
        self.sideLength = sideLength - self.motion_shift_correction_y
        self.rotation = math.degrees(rotation)
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if rotation>0 or sideLength<0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        rotation = -self.rotation/222
        alpha = 0
        alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        framestep = self.simThreadCycleInMs//10
        xtl0 = self.stepLength * (1 - (self.fr1 + self.fr2 + 2 * framestep) / (2*self.fr1+self.fr2+ 2 * framestep)) * 1.5     # 1.5 - podgon
        xtr0 = self.stepLength * (1/2 - (self.fr1 + self.fr2 + 2 * framestep ) / (2*self.fr1+self.fr2+ 2 * framestep))
        dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion forward per framestep
        dy0_typical = self.sideLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion sideways per framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        for iii in range(0,frameNumberPerCycle,framestep):
            if self.glob.SIMULATION == 2 or 5: start1 = time.time()*1000
            if 0<= iii <self.fr1 :                                              # FASA 1
                alpha = alpha01 * (iii/2+0.5*framestep)
                #alpha = alpha01 * iii/2
                S = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
                self.ytr = S - self.d10 + self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                if cycle ==0: continue
                else: dx0 = dx0_typical
                self.xtl = xtl0 - dx0 - dx0 * iii/framestep
                self.xtr = xtr0 - dx0 - dx0 * iii/framestep

            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :                     # FASA 3
                alpha = alpha01 * ((iii-self.fr2)/2+0.5*framestep)
                #alpha = alpha01 * (iii-self.fr2)/2
                S = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
                self.ytr = S - self.d10 - self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                dx0 = dx0_typical
                self.xtl -= dx0
                self.xtr -= dx0

            if self.fr1<= iii <self.fr1+self.fr2:                               # FASA 2
                self.ztr = -self.gaitHeight + self.stepHeight
                if cycle ==0:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep/2
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep #* 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                if iii==self.fr1:
                    self.xtr -= dx0
                    #self.ytr = S - 64 + dy0
                    self.ytr = S - self.d10 + dy0
                elif iii == (self.fr1 +self.fr2 - framestep):
                    self.xtr -= dx0
                    self.ytr = S - self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtr += dx
                    self.ytr = S - 64 + dy0 - dy*self.fr2/(self.fr2- 2 * framestep)*((iii - self.fr1)/2)
                    self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
                self.xtl -= dx0
                self.ytl += dy0

            if 2*self.fr1+self.fr2<= iii :                                         # FASA 4
                self.ztl = -self.gaitHeight + self.stepHeight
                if cycle == number_Of_Cycles - 1:
                    dx0 = dx0_typical * 4 / self.fr2           # 8.75/6
                    dx = (self.stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2- 2 * framestep) *framestep / 1.23076941   # 1.23076941 = podgon
                    if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                        self.ztl = -self.gaitHeight
                        self.ytl = S + self.d10
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep) *framestep # * 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/(self.fr2- 2 * framestep) *framestep
                    dy0 = dy0_typical
                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl -= dx0
                    #self.ytl = S + 64 + dy0
                    self.ytl = S + self.d10 + dy0
                elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.xtl -= dx0
                    self.ytl = S + self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtl += dx
                    self.ytl = S + 64 + dy0 - dy * (iii -(2*self.fr1+self.fr2) )/2
                    self.wr = self.wl = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2- 2 * framestep) *2 - rotation
                self.xtr -= dx0
                self.ytr += dy0
            angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #print('iii = ', iii, 'ytr =', self.ytr, 'ytl =', self.ytl)
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
                        #uprint(self.euler_angle)
                        self.timeElapsed = self.timeElapsed +1
                        #uprint(Dummy_Hposition)
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                            self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                        if self.glob.SIMULATION == 1:
                            self.sim_simxSynchronousTrigger(self.clientID)
                        #disp = []
                        #for i in range(len(angles)):
                        #    pos = int(angles[i]*1698 + 7500)
                        #    #disp.append((self.ACTIVEJOINTS[i],self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos -7500))
                        #    if 0 <= i <= 4 or 11 <= i <= 15:
                        #        disp.append(pos -7500)
                        #print(iii, self.xtr, self.ytr, self.ztr, self.xr, self.yr, self.zr, self.wr, self.xtl, self.ytl, self.ztl, self.xl, self.yl, self.zl, self.wl)
                        #print(iii, self.wr)
                elif self.glob.SIMULATION == 2 or 5:
                    servoSpeedDatas =[]
                    #if iii == 2*self.fr1+2*self.fr2 - framestep or iii == self.fr1 + self.fr2 - framestep:
                    #    servoSpeedDatas.append(self.kondo.ServoData(7, 1, 20))
                    #    servoSpeedDatas.append(self.kondo.ServoData(8, 1, 127))
                    #    servoSpeedDatas.append(self.kondo.ServoData(9, 1, 20))
                    #    servoSpeedDatas.append(self.kondo.ServoData(7, 2, 20))
                    #    servoSpeedDatas.append(self.kondo.ServoData(8, 2, 127))
                    #    servoSpeedDatas.append(self.kondo.ServoData(9, 2, 20))
                    #else:
                    for i in range(7,10):
                        servoSpeedDatas.append(self.kondo.ServoData(i, 1, 127))
                        servoSpeedDatas.append(self.kondo.ServoData(i, 2, 127))
                    self.kondo.setServoSpeed(servoSpeedDatas)
                    servoDatas = []
                    disp = []
                    for i in range(len(angles)):
                        pos = int(angles[i]*1698 + 7500)
                        disp.append((self.ACTIVEJOINTS[i],self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos -7500))
                        servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                    servoDatas = self.reOrderServoData(servoDatas)
                    #start2 = self.pyb.millis()
                    start2 = time.time()*1000
                    a=self.kondo.setServoPos (servoDatas, self.frames_per_cycle)
                    #print('disp[4] = ', disp[4], 'disp[15]=', disp[15])
                    #time1 = self.pyb.elapsed_millis(start1)
                    #time2 = self.pyb.elapsed_millis(start2)
                    time1 = time.time()*1000 - start1
                    time2 = time.time()*1000 - start2
                    #print('calc time =',time1 - time2, 'transfer time =', time2 )
                    #self.pyb.delay(self.frame_delay - time1)
                    if (self.frame_delay - time1) > 0:
                        time.sleep((self.frame_delay - time1)/1000)
                #self.refresh_Orientation()
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        self.local.coord_shift[0] = self.cycle_step_yield*stepLength/64/1000
        if self.first_Leg_Is_Right_Leg:
            self.local.coord_shift[1] = -self.side_step_right_yield * abs(sideLength)/20/1000
        else: self.local.coord_shift[1] = self.side_step_left_yield * abs(sideLength)/20/1000
        self.local.coordinate_record(odometry = True, shift = True)
        #self.first_Leg_Is_Right_Leg = tmp1

    def walk_Final_Pose(self):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        framestep = self.simThreadCycleInMs//10
        for j in range (self.initPoses):
            if self.glob.SIMULATION == 2 or 5: start1 = time.time()*1000
            self.ztr = -self.gaitHeight - (j+1)*(223.0-self.gaitHeight)/self.initPoses
            self.ztl = -self.gaitHeight - (j+1)*(223.0-self.gaitHeight)/self.initPoses
            self.ytr = -self.d10 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
            self.ytl =  self.d10 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
            if j == self.initPoses - 1:
                angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1, hands_on = False)
            else: angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        #self.Dummy_HData.append(Dummy_Hposition)
                        #self.timeElapsed = self.timeElapsed +1
                        # uprint(Dummy_Hposition)
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                            self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                    if self.glob.SIMULATION == 1:
                        self.sim_simxSynchronousTrigger(self.clientID)
                elif self.glob.SIMULATION == 2 or 5:
                    servoDatas = []
                    for i in range(len(angles)):
                        pos = int(angles[i]*1698 + 7500)
                        servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                    servoDatas = self.reOrderServoData(servoDatas)
                    #start2 = self.pyb.millis()
                    start2 = time.time()*1000
                    a=self.kondo.setServoPos (servoDatas, self.frames_per_cycle)
                    #uprint(servoDatas)
                    #uprint(clock.avg())
                    #time1 = self.pyb.elapsed_millis(start1)
                    #time2 = self.pyb.elapsed_millis(start2)
                    time1 = time.time()*1000 - start1
                    time2 = time.time()*1000 - start2
                    time.sleep((self.frame_delay - time1)/1000)
                    #self.pyb.delay(self.frame_delay - time1)

    def kick(self, first_Leg_Is_Right_Leg, small = False):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        gaitHeight = 210
        stepHeight = 55
        stepLength = 64
        kick_size = 70
        if small : kick_size = -10
        tmp1 = self.first_Leg_Is_Right_Leg
        self.first_Leg_Is_Right_Leg = first_Leg_Is_Right_Leg
        tmp = self.gaitHeight
        self.gaitHeight = gaitHeight
        self.walk_Initial_Pose()
        alpha = 0
        alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        framestep = self.simThreadCycleInMs//10
        dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        for iii in range(0,frameNumberPerCycle,framestep):
            if self.glob.SIMULATION == 2 or 5: start1 = time.time()*1000
            if 0<=iii <self.fr1:
                alpha = alpha01 * (iii/2+0.5*framestep)
                S = (self.amplitude/2 )*math.cos(alpha)
                self.ytr = S - self.d10
                self.ytl = S + self.d10
                self.ztl = -gaitHeight
                self.ztr = -gaitHeight
                continue
            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :
                alpha = alpha01 * ((iii-self.fr2)/2+0.5*framestep)
                S = (self.amplitude/2)*math.cos(alpha)
                self.ytr = S - self.d10
                self.ytl = S + self.d10
                self.ztl = -gaitHeight
                self.ztr = -gaitHeight
                dx0 = dx0_typical
                self.xtl -= dx0
                self.xtr -= dx0
            if self.fr1<= iii <self.fr1+self.fr2:
                self.ztr = -gaitHeight + stepHeight
                dx = stepLength/2/self.fr2*2
                dx0 = stepLength/(2*self.fr1+self.fr2+4)*framestep
                if iii==self.fr1:
                    self.xtr -= dx0
                    self.ytr = S - 64
                elif iii == (self.fr1 +self.fr2 - 2):
                    self.xtr -= dx0
                    self.ytr = S - 64
                else:
                    self.xtr += dx*self.fr2/(self.fr2-2 * framestep)
                    self.ytr = S - 64
                if iii == self.fr1 +self.fr2 - 10: self.xtr += kick_size
                if iii == self.fr1 +self.fr2 - 4: self.xtr -= kick_size
                self.xtl -= dx0
            if 2*self.fr1+self.fr2<= iii :
                self.ztl = -gaitHeight + stepHeight
                dx0 = dx0_typical * 4 / self.fr2           # 8.75/6
                dx = (stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2 - 2 * framestep) * framestep
                if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.ztl = -gaitHeight
                    self.ytl = S + self.d10

                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl -= dx0
                    self.ytl = S + 64
                elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.xtl -= dx0
                    self.ytl = S + 64
                else:
                    self.xtl += dx
                    self.ytl = S + 64
                self.xtr -= dx0
            angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        #uprint(self.euler_angle)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        self.timeElapsed = self.timeElapsed +1
                        #uprint(Dummy_Hposition)
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                            self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                        if self.glob.SIMULATION == 1:
                            self.sim_simxSynchronousTrigger(self.clientID)
                elif self.glob.SIMULATION == 2 or 5:
                    servoDatas = []
                    disp = []
                    for i in range(len(angles)):
                        pos = int(angles[i]*1698 + 7500)
                        disp.append((self.ACTIVEJOINTS[i],self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos -7500))
                        servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                    servoDatas = self.reOrderServoData(servoDatas)
                    #start2 = self.pyb.millis()
                    a=self.kondo.setServoPos (servoDatas,self.frames_per_cycle)
                    #uprint(disp)
                    #time2 = self.pyb.elapsed_millis(start2)
                    #time1 = self.pyb.elapsed_millis(start1)
                    time1 = time.time()*1000 - start1
                    time.sleep(self.frame_delay - time1)
                    #self.pyb.delay(self.frame_delay - time1)
                #self.refresh_Orientation()
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        self.walk_Final_Pose()
        self.local.coord_shift[0] = self.first_step_yield/2000
        self.local.coord_shift[1] = 0
        self.local.coordinate_record(odometry = True, shift = True)
        self.gaitHeight = tmp
        self.first_Leg_Is_Right_Leg = tmp1

    def refresh_Orientation(self):
        if self.glob.SIMULATION == 2 or 5:
            yaw, roll, pitch = self.imu.euler()
            if 0<= yaw < 180: yaw = -yaw
            if 180<= yaw <=360: yaw = 360 -yaw
            self.euler_angle = [yaw, roll, pitch]
            self.euler_angle[0] = math.radians(self.euler_angle[0])
            self.euler_angle[0]-= self.direction_To_Attack
        else:
            returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
            self.euler_angle = list(self.quaternion_to_euler_angle(Dummy_Hquaternion))
            self.euler_angle[0] = math.radians(self.euler_angle[0])
            self.euler_angle[0] -= self.direction_To_Attack

if __name__=="__main__":
    print('This is not main module!')


