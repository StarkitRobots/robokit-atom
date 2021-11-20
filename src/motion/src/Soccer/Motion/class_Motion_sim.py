#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if current_work_directory.find('Soccer') >= 0:
    current_work_directory = current_work_directory[:-14]
if sys.version != '3.4.0':
    current_work_directory += '/'
    with open(current_work_directory + "simulator_lib_directory.txt", "r") as f:
        simulator_lib_directory = f.read()
    simulator_lib_directory = simulator_lib_directory.replace('\\', '/')
    sys.path.append(simulator_lib_directory)
    import random
    import sim, threading
else:
    import starkit
    sys.path.append('/')

sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
sys.path.append( current_work_directory + 'Soccer/Vision/')
sys.path.append( current_work_directory + 'Soccer/Localisation/')
sys.path.append( current_work_directory + 'Soccer/Localisation/PF/')


from class_Motion import *
from class_Motion_real import Motion_real
from compute_Alpha_v3 import Alpha

class Transfer_Data():
    def __init__(self):
        self.stop_Flag = False
        self.finish_Flag = False
        self.pause = False
        self.stop = 0
        self.finish = 0


def sim_Enable(ip_address, port):
    simThreadCycleInMs = 2
    uprint ('Simulation started')
    #sim.simxFinish(-1) # just in case, close all opened connections
    clientID = sim.simxStart(ip_address, port, True, True, 5000, simThreadCycleInMs)
    if clientID != -1:
        uprint ('Connected to remote API server')
    else:
        uprint ('Failed connecting to remote API server')
        uprint ('Program ended')
        exit(0)
    return clientID

def simulation_Trigger_Accumulator(clientIDs, events, transfer_Datas, lock):
    while(True):
        finish = False
        while(True):
            for transfer_Data in transfer_Datas:
                if transfer_Data.stop_Flag == True:
                    finish = True
                    for transfer_Data in transfer_Datas:
                        if transfer_Data.finish_Flag == False: finish = False
                    for transfer in transfer_Datas:
                        transfer.stop_Flag = True
                    break
            event_flag = True
            for i in range(len(events)):
                if (not events[i].is_set()): event_flag = False
                if transfer_Datas[i].finish_Flag == True:
                    event_flag = True
                    break
            if event_flag == True: break
            time.sleep(0.001)
        lock.acquire()
        lock.release()
        for clientID in clientIDs:
            sim.simxSynchronousTrigger(clientID)
        for event in events: event.clear()
        if finish == True:
            for clientID in clientIDs:
                sim.simxFinish(clientID)
            break
    pass


class Motion_sim(Motion_real):
    def __init__(self, glob, vision, clientID , motion_EventID,  lock, transfer_Data, numberOfRobots, robot_Number = ''):
        self.FRAMELENGTH = 0.02
        import random as random
        self.random = random
        import sim as vr
        self.sim = vr
        import numpy as np
        self.np = np
        import matplotlib.pyplot as plt
        self.plt = plt
        import cv2 as cv2
        self.cv2 = cv2
        import reload as re
        self.re = re
        import msvcrt as ms
        self.ms = ms
        self.Dummy_HData =[]
        self.BallData =[]
        self.timeElapsed = 0
        self.trims = []
        self.jointHandle = []
        self.Dummy_HHandle = 0
        self.Dummy_1Handle = 0
        self.BallHandle = 0
        self.VisionHandle = 0
        self.Ballposition = []
        self.transfer_Data = transfer_Data
        self.lock = lock
        self.motion_Event = motion_EventID
        self.clientID = clientID
        self.robot_Number = robot_Number
        self.Vision_Sensor_Display_On = True
        self.sim_step_counter = 0
        self.numberOfRobots = numberOfRobots
        #self.SIM_OPTION = 'SURROGAT'  # other variants: 'KONDOv23',  'ODE'
        super().__init__(glob, vision)
        with open(current_work_directory + "Init_params/Sim/" + self.glob.SIM_OPTION + "/Sim_calibr.json", "r") as f:
            data1 = json.loads(f.read())
        self.neck_calibr = data1['neck_calibr']
        self.neck_play_pose = data1['neck_play_pose']
        self.neck_tilt = self.neck_calibr

    def wait_sim_step(self):
        while True:
            self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_buffer)
            tim = self.sim.simxGetLastCmdTime(self.clientID)
            #print ('Simulation time: ', tim)
            if tim > self.sim_step_counter:
                self.sim_step_counter = tim 
                break
            time.sleep(0.001)
            if self.transfer_Data.stop > 0:
                self.transfer_Data.stop += 1
                self.sim_Stop()
                while True:
                    if self.transfer_Data.stop == self.numberOfRobots:
                        self.sim_Disable()
                        sys.exit(0)
                    time.sleep(0.1)

    def getSimTime(self):
        inputInts=[]
        inputFloats=[]
        inputStrings=[]
        inputBuffer=bytearray()
        name = 'Telo_Surrogat'+ self.robot_Number
        self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
        res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(self.clientID,name,sim.sim_scripttype_childscript,
                        'getSimTime',inputInts,inputFloats,inputStrings,inputBuffer,sim.simx_opmode_blocking)
        self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)   
        if len(retFloats) == 0: return 0
        return retFloats[0]

    def sim_simxSynchronousTrigger(self, clientID):
        if  self.glob.SIMULATION == 1 :
            if self.transfer_Data.stop > 0:
                    self.transfer_Data.stop += 1
                    self.sim_Stop()
                    while True:
                        if self.transfer_Data.stop == self.numberOfRobots:
                            self.sim_Disable()
                            sys.exit(0)
                        time.sleep(0.1)
        if  self.glob.SIMULATION == 3 : 
            self.wait_sim_step()
            return
        if self.motion_Event == 0:
            self.sim.simxSynchronousTrigger(clientID)
        else:
            self.motion_Event.set()
            while (self.motion_Event.is_set()): time.sleep(0.001)

    def vision_Sensor_Get_Image(self):
        returnCode, resolution, image_Data = self.sim.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.sim.simx_opmode_buffer)
        nuimg = self.np.array(image_Data, dtype=self.np.uint8)
        nuimg.shape = (resolution[1],resolution[0],3)
        nuimg1 = self.cv2.cvtColor(nuimg, self.cv2.COLOR_RGB2BGR)
        img = self.np.flip(nuimg1, 1)
        return img

    def vision_Sensor_Display(self, img):
        if self.Vision_Sensor_Display_On:
            self.cv2.imshow('Vision Sensor'+ self.robot_Number, img)
            self.cv2.waitKey(10) & 0xFF
            #if self.robot_Number != '':
            #    self.cv2.waitKey(10) & 0xFF
            #else:
            #    res = self.cv2.waitKey(0)
            #    if res == 115:
            #        print('you have pressed "s"')
            #        token = str(int(self.random.random()*10000))
            #        filename = current_work_directory + "Soccer/CameraStill/VisionSensor" + token + '.png'
            #        isWritten = self.cv2.imwrite(filename, img)

    def simulateMotion(self, number = 0, name = ''):
        #mot = [(0,'Initial_Pose'),(1,0),(2,0),(3,0),(4,0),(5,'Get_Up_Left'),
        #   (6,'Soccer_Get_UP_Stomach_N'),(7,0),(8,'Soccer_Walk_FF'),(9,0),(10,0),
        #   (11,0),(12,0),(13,0),(14,'Soccer_Small_Jump_Forward'),(15,0),
        #   (16,0),(17,0),(18,'Soccer_Kick_Forward_Right_Leg'),(19,'Soccer_Kick_Forward_Left_Leg'),(20,0),
        #   (21,'Get_Up_From_Defence'),(22,0),(23,'PanaltyDefenceReady_Fast'), (24,'PenaltyDefenceF'),(25,0),
        #   (26,0),(27,0),(28,0),(29.0),(30,'Soccer_Walk_FF0'),
        #   (31,'Soccer_Walk_FF1'), (32,'Soccer_Walk_FF2'), (33,'Soccer_Get_UP_Stomach'), (34,'Soccer_Get_UP_Face_Up'),
        #   (35,'Get_Up_Right'), (36,'PenaltyDefenceR'), (37,'PenaltyDefenceL')]
        # start the simulation
        if number > 0 and name == '': name = self.MOTION_SLOT_DICT[number]
        with open(current_work_directory + "Soccer/Motion/motion_slots/" + name + ".json", "r") as f:
            slots = json.loads(f.read())
        mot_list = slots[name]
        i=0
        for i in range(len(mot_list)):
            if  self.falling_Flag ==3: return
            activePoseOld = []
            for ind in range(len(self.activePose)): activePoseOld.append(self.activePose[ind])
            self.activePose =[]
            for j in range(len(self.ACTIVEJOINTS) - 2):
                    self.activePose.append(0.017*mot_list[i][j+1]*0.03375)
            pulseNum = int(mot_list[i][0]*self.FRAMELENGTH * 1000 / self.simThreadCycleInMs)
            for k in range (pulseNum):
                if self.glob.SIMULATION == 3: self.wait_sim_step()
                self.sim.simxPauseCommunication(self.clientID, True)
                for j in range(len(self.ACTIVEJOINTS) - 2):
                    tempActivePose = activePoseOld[j]+(self.activePose[j]-activePoseOld[j])*k/pulseNum
                    returnCode = self.sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                                 tempActivePose*self.FACTOR[j] +self.trims[j], self.sim.simx_opmode_streaming)
                self.sim.simxPauseCommunication(self.clientID, False)
                if self.glob.SIMULATION == 1:
                    self.sim_simxSynchronousTrigger(self.clientID)
        return


    def sim_Start(self):
        #uprint ('Simulation started')
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
            #self.sim.simxFinish(-1) # just in case, close all opened connections
            #self.clientID=self.sim.simxStart('127.0.0.1',19997,True,True,5000,self.simThreadCycleInMs) # Connect to V-REP
            #if self.clientID!=-1:
            #    uprint ('Connected to remote API server')
            #else:
            #    uprint ('Failed connecting to remote API server')
            #    uprint ('Program ended')
            #    exit(0)
            ## Collect Joint Handles and trims from model
            returnCode, self.Dummy_HHandle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy_H'+ self.robot_Number, self.sim.simx_opmode_blocking)
            returnCode, self.Dummy_1Handle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy1' + self.robot_Number, self.sim.simx_opmode_blocking)
            returnCode, self.BallHandle = self.sim.simxGetObjectHandle(self.clientID, 'Ball', self.sim.simx_opmode_blocking)
            returnCode, self.VisionHandle = self.sim.simxGetObjectHandle(self.clientID, 'Vision_sensor' + self.robot_Number, self.sim.simx_opmode_blocking)
            returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_streaming)
            returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
            returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
            returnCode, Dummy_1position= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
            returnCode, resolution, image_Data = self.sim.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.sim.simx_opmode_streaming)
            returnCode, Camera_quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.VisionHandle , -1, self.sim.simx_opmode_streaming)
            #uprint(Dummy_Hquaternion)
            for i in range(len(self.ACTIVEJOINTS)):
                returnCode, handle= self.sim.simxGetObjectHandle(self.clientID, self.ACTIVEJOINTS[i] + self.robot_Number, self.sim.simx_opmode_blocking)
                self.jointHandle.append(handle)
                returnCode, position= self.sim.simxGetJointPosition(self.clientID, handle, self.sim.simx_opmode_blocking)
                self.trims.append(position)
                self.activePose.append(position)
            if self.glob.SIMULATION == 1:
                self.sim.simxSynchronous(self.clientID,True)
            if self.glob.SIMULATION == 3:
                self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_streaming)

    def sim_Progress(self,simTime):  # simTime in seconds
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
            for i in range(int(simTime*1000//self.simThreadCycleInMs)):
                returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                self.Dummy_HData.append(Dummy_Hposition)
                returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                self.BallData.append(self.Ballposition)
                returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                #uprint(quaternion_to_euler_angle(Dummy_Hquaternion))
                self.timeElapsed = self.timeElapsed +1
                self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                if self.glob.SIMULATION == 1 : self.sim_simxSynchronousTrigger(self.clientID)
                if self.glob.SIMULATION == 3 : 
                    time.sleep(0.005)
                    self.wait_sim_step() 

    def sim_Stop(self):
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
            self.sim.simxStopSimulation(self.clientID,self.sim.simx_opmode_oneshot)
                    # return to initial pose
            for j in range(len(self.ACTIVEJOINTS)):
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                   returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                self.jointHandle[j] , self.trims[j], self.sim.simx_opmode_oneshot)
                else: returnCode = self.sim.simxSetJointPosition(self.clientID,
                                   self.jointHandle[j] , self.trims[j], self.sim.simx_opmode_oneshot)
    def print_Diagnostics(self):
        Dummy_HDataX =[]
        Dummy_HDataY =[]
        Dummy_HDataZ =[]
        for i in range (self.timeElapsed):
            Dummy_HDataX.append( self.Dummy_HData[i][0])
            Dummy_HDataY.append(self.Dummy_HData[i][1])
            Dummy_HDataZ.append(self.Dummy_HData[i][2])
        BallDataX =[]
        BallDataY =[]
        BallDataZ =[]
        for i in range (self.timeElapsed):
            BallDataX.append( self.BallData[i][0])
            BallDataY.append(self.BallData[i][1])
            BallDataZ.append(self.BallData[i][2])
        uprint('exitFlag' ,self.exitFlag)
        rng = self.np.arange(self.timeElapsed)
        fig,ax = self.plt.subplots(figsize=(10,6))
        self.plt.plot(rng,Dummy_HDataX, label = 'Body X')
        self.plt.plot(rng,Dummy_HDataY, label = 'Body Y')
        self.plt.plot(rng,Dummy_HDataZ, label = 'Body Z')
        self.plt.plot(rng,BallDataX, label = 'Ball X')
        self.plt.plot(rng,BallDataY, label = 'Ball Y')
        self.plt.plot(rng,BallDataZ, label = 'Ball Z')
        ax.legend(loc='upper left')
        ax.grid(True)
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
            self.plt.show()
            #break
        uprint('exitFlag' ,self.exitFlag)

    def sim_Disable(self):            # Now close the connection to V-REP:
        time.sleep(0.2)
        #self.sim.simxFinish(self.clientID)
        self.transfer_Data.finish_Flag = True


if __name__=="__main__":
    print('This is not main module!')


