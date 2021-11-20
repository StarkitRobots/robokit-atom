import sys
import os
import math
import json
import time


current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')

if sys.version == '3.8.5 (default, Jan 27 2021, 15:41:15) \n[GCC 9.3.0]':
    current_work_directory += '/'
    # will be running on openMV
    SIMULATION = 5                                         # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on openMV
else:
    # will be running on desktop computer
    current_work_directory += '/'
    import threading
    SIMULATION = 3 
   
sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
sys.path.append( current_work_directory + 'Soccer/Vision/')
sys.path.append( current_work_directory + 'Soccer/Localisation/')
sys.path.append( current_work_directory + 'Soccer/Localisation/PF/')



def set_flags(flags, prefix = "v4l2-ctl --set-ctrl="):
	for k, v in flags.items():
		os.system(prefix + k + "=" + str(v))

flags = {"exposure_auto"                  : 1,
         "white_balance_temperature_auto" : 0,
         "exposure_auto_priority"         : 0,
         "backlight_compensation"         : 0,
         "focus_auto"                     : 0}

set_flags(flags)

try:
    if SIMULATION == 2 or SIMULATION == 5:
        
        from Serial_arduino import Serial_arduino
        button = Serial_arduino()
        pressed_button = button.get_button()
        print("pressed_button = ", pressed_button)

        #if pressed_button == 9:
        #    execfile("threshold_tuner_server.py")

        from strategy import Player

        with open(current_work_directory + "Init_params/Real/Real_landmarks.json", "r") as f:
            landmarks = json.loads(f.read())
        subrole = ' '
        if pressed_button == 1:
            role = 'forward'
            start_point = [-0.4, 0, 0]
        if pressed_button == 2:
            role = 'forward'
            subrole = ' Rignt'
            x = -(landmarks['FIELD_LENGTH'] / 2 - 0.9)
            y = -landmarks['FIELD_WIDTH'] / 2
            start_point = [x, y, math.pi/2]
        if pressed_button == 3:
            role = 'forward'
            subrole = ' Left'
            x = -(landmarks['FIELD_LENGTH'] / 2 - 0.9)
            y = landmarks['FIELD_WIDTH'] / 2
            start_point = [x, y, math.pi/2]
        if pressed_button == 4:
            role = 'penalty_Shooter'
            start_point = [0.3, 0, 0]
        if pressed_button == 5:
            role = 'penalty_Goalkeeper'
            start_point = [-landmarks['FIELD_LENGTH'] / 2, 0, 0]
        if pressed_button == 6:
            role = 'goalkeeper'
            start_point = [-landmarks['FIELD_LENGTH'] / 2, 0, 0]
        if pressed_button == 7:
            role = 'run_test'
            start_point = [0.0, 0.0, 0]
        if pressed_button == 8:
            role = 'dance'
            start_point = [0.0, 0.0, 0]
        player = Player(role)
        print( role, subrole, ' start_point = ', start_point)
        player.real(start_point, button)
    else:
        from strategy import Player
        from class_Motion_sim import sim_Enable, Transfer_Data, simulation_Trigger_Accumulator
        clientID = []
        transfer_Datas =[]
        robots_Number = 1
        robot_IDs = [ '', '#0', '#1', '#2']
        initial_coords = [[0, 0, 0], [ -1.8, 0, 0], [-1.1, 0, 0], [-1.8, 0, 0]]
        clientID.append(sim_Enable('127.0.0.1', -19997))
        print('clientID1 =', clientID[0])
        if robots_Number > 1:
            clientID.append(sim_Enable('127.0.0.2', -19998))
            print('clientID2 =', clientID[1])
        if robots_Number > 2:
            clientID.append(sim_Enable('127.0.0.3', -19999))
            print('clientID3 =', clientID[2])
        if robots_Number > 3:
            clientID.append(sim_Enable('127.0.0.4', -20000))
            print('clientID4 =', clientID[3])

        events = []
        t = []
        m =[]
        lock = threading.Lock()
        transfer_Data = Transfer_Data()
        for i in range(robots_Number):
            if i == 0 or i == 3:
                m.append(Player('forward'))  #goalkeeper, forward_v2, run_test, penalty_Shooter, rotation_test, penalty_Goalkeeper, spot_walk, dance
            if i == 1:
                m.append(Player('goalkeeper'))
            if i == 2:
                m.append(Player('forward_v2'))
            events.append(threading.Event())
            transfer_Datas.append(transfer_Data)
        if  SIMULATION == 1 :
            t0 = threading.Thread( target = simulation_Trigger_Accumulator, args=(clientID, events, transfer_Datas, lock))
        for i in range(robots_Number):
            t.append(threading.Thread( target = m[i].simulation, args=(clientID[i], events[i], robots_Number, robot_IDs[i], lock, transfer_Datas[i], initial_coords[i])))
        if  SIMULATION == 1 :
            t0.setDaemon(True)
            t0.start()
        for i in range(robots_Number): t[i].start()


except Exception as e:
    if SIMULATION == 2:
        f = open("Error.txt",'w')
        raise e
        #sys.print_exception(e,f)
        f.close()
        #sys.print_exception(e,sys.stdout)
    else:
        raise e
        print(e)
