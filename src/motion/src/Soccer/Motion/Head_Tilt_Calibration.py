import sys, os
import math, time, json

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if current_work_directory.find('Soccer') >= 0:
    current_work_directory = current_work_directory[:-14]
if sys.version != '3.4.0':
    current_work_directory += '/'
    import random
    SIMULATION = 1
else:
    SIMULATION = 2
    import pyb


sys.path.append( current_work_directory + '/')
sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
sys.path.append( current_work_directory + 'Soccer/Vision/')
sys.path.append( current_work_directory + 'Soccer/Localisation/')
sys.path.append( current_work_directory + 'Soccer/Localisation/PF/')


from class_Glob import Glob
from class_Vision import Vision
from class_Local import *
if SIMULATION == 2:
    from class_Motion_real import Motion_real as Motion
else:
    from class_Motion_sim import *
    from class_Motion_sim import Motion_sim as Motion


             # 0 - Simulation without physics, 1 - Simulation with physics, 2 - live on openMV

if SIMULATION == 2:
    from button_test import Button_Test
    button = Button_Test() 
    glob = Glob(SIMULATION, current_work_directory, particles_number = 100)
    vision = Vision(glob)
    motion = Motion(glob, vision)
    motion.push_Button(button)
    pyb.delay(2000)
    motion.activation()
    motion.head_Tilt_Calibration()
else:
    def head_tilt_calibration_in_sim(clientID, event, lock, transfer_Data):
        glob = Glob(SIMULATION, current_work_directory)
        vision = Vision(glob)
        motion = Motion(glob, vision, clientID , event,  lock, transfer_Data, 1, robot_Number = '')
        motion.slowTime = 0.0
        motion.sim_Start()
        motion.activation()
        motion.Vision_Sensor_Display_On = True
        motion.sim_Start()
        motion.head_Tilt_Calibration()
        motion.sim_Progress(1)
        motion.sim_Stop()
        motion.sim_Disable()
    import threading
    event = threading.Event()
    events = [event]
    transfer_Data = Transfer_Data()
    transfer_Datas = [transfer_Data]
    lock = threading.Lock()
    clientID = sim_Enable('127.0.0.1', -20000)
    clientIDs = [clientID]
    t0 = threading.Thread( target = simulation_Trigger_Accumulator, args=(clientIDs, events, transfer_Datas, lock))
    t = threading.Thread( target = head_tilt_calibration_in_sim, args=(clientID, event, lock, transfer_Data))
    t0.setDaemon(True)
    t0.start()
    t.start()
    
