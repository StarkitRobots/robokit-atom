

import sys
import os
import math
import json
import time


current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if sys.version == '3.8.5 (default, Jan 27 2021, 15:41:15) \n[GCC 9.3.0]':
    # will be running on openMV
    #mport pyb
    current_work_directory += '/'
    SIMULATION = 5                                        # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on openMV
else:
    # will be running on desktop computer
    current_work_directory += '/'
    SIMULATION = 3                                          # 3 - Simulation streaming with physics

from class_Vision import Vision
from class_Local import *
from class_Glob import Glob
import ParticleFilter as pf
if SIMULATION == 2 or 5:
    from class_Motion_real import Motion_real as Motion
else:
    from class_Motion import *
    from class_Motion_sim import*
    from class_Motion_sim import Motion_sim as Motion



""" printing utility uprint prints to console and to output.txt file simultaneously with the same syntax as print """
def uprint(*text):
    #with open(current_work_directory + "Soccer/log/output.txt",'a') as f:
    #    print(*text, file = f)
    print(*text )

class GoalKeeper:
    def __init__(self, motion, local, glob):
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0

    def turn_Face_To_Guest(self):
        if self.glob.pf_coord[0] < 0:
            self.motion.turn_To_Course(0)
            self.direction_To_Guest = 0
            return
        elif self.glob.pf_coord[0] > 0.8 and abs(self.glob.pf_coord[1]) > 0.6:
            self.direction_To_Guest = math.atan(-self.glob.pf_coord[1]/(1.8-self.glob.pf_coord[0]))
            self.motion.turn_To_Course(self.direction_To_Guest)
        elif self.glob.pf_coord[0] < 1.5 and abs(self.glob.pf_coord[1]) < 0.25:
            if (1.8-self.glob.ball_coord[0]) == 0: self.direction_To_Guest = 0
            else: self.direction_To_Guest = math.atan((0.4* (round(pf.random(),0)*2 - 1)-
                                                       self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
            self.motion.turn_To_Course(self.direction_To_Guest)
            return
        else:
            self.direction_To_Guest = math.atan(-self.glob.pf_coord[1]/(2.8-self.glob.pf_coord[0]))
            self.motion.turn_To_Course(self.direction_To_Guest)

    def goto_Center(self):                      #Function for reterning to center position
        uprint('Function for reterning to center position')
        if self.local.call_Par_Filter.pf.consistency < 0.5: self.motion.localisation_Motion()
        player_X_m = self.glob.pf_coord[0]
        player_Y_m = self.glob.pf_coord[1]
        duty_position_x = - self.glob.landmarks['FIELD_LENGTH']/2 + 0.2
        distance_to_target = math.sqrt((duty_position_x -player_X_m)**2 + (0 - player_Y_m)**2 )
        if distance_to_target > 0.5 :
            target_in_front_of_duty_position = [duty_position_x + 0.15, 0]
            if distance_to_target > 1: stop_Over = True
            else: stop_Over = False
            self.motion.far_distance_plan_approach(target_in_front_of_duty_position, self.direction_To_Guest, stop_Over = stop_Over)
        else:
            if (duty_position_x -player_X_m)==0:
                alpha = math.copysign(math.pi/2, (0 - player_Y_m) )
            else:
                if (duty_position_x - player_X_m)> 0: alpha = math.atan((0 - player_Y_m)/(duty_position_x -player_X_m))
                else: alpha = math.atan((0 - player_Y_m)/(duty_position_x - player_X_m)) + math.pi
            napravl = alpha - self.motion.imu_body_yaw()
            dist_mm = distance_to_target * 1000
            self.motion.near_distance_omni_motion(dist_mm, napravl)
        self.turn_Face_To_Guest()

    def find_Ball(self):
        fast_Reaction_On=True
        if self.local.call_Par_Filter.pf.consistency < 0.5: fast_Reaction_On = False
        if self.glob.ball_coord[0] <= 0: fast_Reaction_On=True
        success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On=fast_Reaction_On)
        #uprint ( 'dist = ', dist, 'napravl =', napravl)
        return success_Code, dist, napravl, speed

    def ball_Speed_Dangerous(self):
        pass
    def fall_to_Defence(self):
        uprint('fall to defence')
    def get_Up_from_defence(self):
        uprint('up from defence')
    def scenario_A1(self, dist, napravl):#The robot knock out the ball to the side of the enemy
        uprint('The robot knock out the ball to the side of the enemy')
        for i in range(10):
            if dist > 0.5 :
                if dist > 1: stop_Over = True
                else: stop_Over = False
                self.motion.far_distance_plan_approach(self.glob.ball_coord, self.direction_To_Guest, stop_Over = stop_Over)
            self.turn_Face_To_Guest()
            success_Code = self.motion.near_distance_ball_approach_and_kick(self.direction_To_Guest)
            if success_Code == False and self.motion.falling_Flag != 0: return
            if success_Code == False : break
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = False)
            if dist > 1 : break
        target_course1 = self.glob.pf_coord[2] +math.pi
        self.motion.turn_To_Course(target_course1)
        self.goto_Center()

    def scenario_A2(self, dist, napravl):#The robot knock out the ball to the side of the enemy
        uprint('The robot knock out the ball to the side of the enemy')
        self.scenario_A1( dist, napravl)

    def scenario_A3(self, dist, napravl):#The robot knock out the ball to the side of the enemy
        uprint('The robot knock out the ball to the side of the enemy')
        self.scenario_A1( dist, napravl)

    def scenario_A4(self, dist, napravl):#The robot knock out the ball to the side of the enemy
        uprint('The robot knock out the ball to the side of the enemy')
        self.scenario_A1( dist, napravl)

    def scenario_B1(self):#the robot moves to the left and stands on the same axis as the ball and the opponents' goal
        uprint('the robot moves to the left 4 steps')
        if self.glob.ball_coord[1] > self.glob.pf_coord[1]:
            if self.glob.ball_coord[1] > 0.4: 
                if self.glob.pf_coord[1] < 0.4:
                    self.motion.near_distance_omni_motion( 1000*(0.4 - self.glob.pf_coord[1]), math.pi/2)
            else:
                self.motion.near_distance_omni_motion( 1000*(self.glob.ball_coord[1] - self.glob.pf_coord[1]), math.pi/2)
        self.turn_Face_To_Guest()

    def scenario_B2(self):#the robot moves to the left and stands on the same axis as the ball and the opponents' goal
        uprint('the robot moves to the left 4 steps')
        self.scenario_B1()

    def scenario_B3(self):#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
        uprint('the robot moves to the right 4 steps')
        #self.motion.first_Leg_Is_Right_Leg = True
        #self.motion.near_distance_omni_motion( 110, -math.pi/2)
        if self.glob.ball_coord[1] < self.glob.pf_coord[1]:
            if self.glob.ball_coord[1] < -0.4: 
                if self.glob.pf_coord[1] > -0.4:
                    self.motion.near_distance_omni_motion( 1000*(0.4 + self.glob.pf_coord[1]), -math.pi/2)
            else:
                self.motion.near_distance_omni_motion( 1000*(-self.glob.ball_coord[1] + self.glob.pf_coord[1]), -math.pi/2)
        self.turn_Face_To_Guest()

    def scenario_B4(self):#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
        uprint('the robot moves to the right 4 steps')
        self.scenario_B3()

class Forward:
    def __init__(self, motion, local, glob):
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0

    def dir_To_Guest(self):
        if self.glob.ball_coord[0] < 0:
            self.direction_To_Guest = 0
        elif self.glob.ball_coord[0] > 0.8 and abs(self.glob.ball_coord[1]) > 0.6:
            self.direction_To_Guest = math.atan(-self.glob.ball_coord[1]/(1.8-self.glob.ball_coord[0]))
        elif self.glob.ball_coord[0] < 1.5 and abs(self.glob.ball_coord[1]) < 0.25:
            if (1.8-self.glob.ball_coord[0]) == 0: self.direction_To_Guest = 0
            else:
                if abs(self.glob.ball_coord[1]) > 0.2:
                    self.direction_To_Guest = math.atan((math.copysign(0.2, self.glob.ball_coord[1])-
                                                       self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
                else:
                    self.direction_To_Guest = math.atan((0.2* (round(pf.random(),0)*2 - 1)-
                                                       self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
        else:
            self.direction_To_Guest = math.atan(-self.glob.pf_coord[1]/(2.8-self.glob.pf_coord[0]))
        return self.direction_To_Guest

    def turn_Face_To_Guest(self):
        self.dir_To_Guest()
        self.motion.turn_To_Course(self.direction_To_Guest)

class Forward_Vector_Matrix:
    def __init__(self, motion, local, glob):
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0
        self.kick_Power = 1

    def dir_To_Guest(self):
        if abs(self.glob.ball_coord[0])  >  self.glob.landmarks["FIELD_LENGTH"] / 2:
            ball_x = math.copysign(self.glob.landmarks["FIELD_LENGTH"] / 2, self.glob.ball_coord[0])
        else: ball_x = self.glob.ball_coord[0]
        if abs(self.glob.ball_coord[1])  >  self.glob.landmarks["FIELD_WIDTH"] / 2:
            ball_y = math.copysign(self.glob.landmarks["FIELD_WIDTH"] / 2, self.glob.ball_coord[1])
        else: ball_y = self.glob.ball_coord[1]
        col = math.floor((ball_x + self.glob.landmarks["FIELD_LENGTH"] / 2) / (self.glob.landmarks["FIELD_LENGTH"] / self.glob.COLUMNS))
        row = math.floor((- ball_y + self.glob.landmarks["FIELD_WIDTH"] / 2) / (self.glob.landmarks["FIELD_WIDTH"] / self.glob.ROWS))
        if col >= self.glob.COLUMNS : col = self.glob.COLUMNS - 1
        if row >= self.glob.ROWS : row = self.glob.ROWS -1
        self.direction_To_Guest = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2 + 1] / 40
        self.kick_Power = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2]
        print('direction_To_Guest = ', math.degrees(self.direction_To_Guest))
        return self.direction_To_Guest

    def turn_Face_To_Guest(self):
        self.dir_To_Guest()
        self.motion.turn_To_Course(self.direction_To_Guest)

class Player():
    def __init__(self, role):
        #self.clock = time.clock()
        #self.clock.tick()
        self.role = role   #'goalkeeper' # 'penalty_Goalkeeper', 'forward', 'forward1', 'penalty_Shooter'
        self.glob = Glob(SIMULATION, current_work_directory, particles_number = 100)
        self.vision = Vision(self.glob)
        self.motion = None
        self.local = None
        self.g = None
        self.f = None
        

    def simulation(self, clientID , motion_EventID, numberOfRobots, robot_Number, lock, transfer_Data, initial_coord):
 
        self.motion = Motion(self.glob, self.vision, clientID , motion_EventID, lock, transfer_Data, numberOfRobots, robot_Number = robot_Number)
        self.motion.sim_Start()
        self.motion.Vision_Sensor_Display_On = False
        self.common_init(initial_coord)
        #eval('self.' + self.role + '_main_cycle()')
        if self.role == 'goalkeeper': self.goalkeeper_main_cycle()
        if self.role == 'penalty_Goalkeeper': self.penalty_Goalkeeper_main_cycle()
        if self.role == 'side_to_side': self.side_to_side_main_cycle()
        if self.role == 'forward': self.forward_main_cycle()
        if self.role == 'forward_v2': self.forward_v2_main_cycle()
        if self.role == 'forward1': self.forward1_main_cycle()
        if self.role == 'penalty_Shooter': self.penalty_Shooter_main_cycle()
        if self.role == 'run_test': self.run_test_main_cycle(4)
        if self.role == 'spot_walk': self.spot_walk_main_cycle(3)
        if self.role == 'rotation_test': self.rotation_test_main_cycle()
        if self.role == 'sidestep_test': self.sidestep_test_main_cycle()
        if self.role == 'obstacle_runner': self.obstacle_runner_main_cycle()
        if self.role == 'dribbling': self.dribbling_main_cycle()
        if self.role == 'ball_moving': self.ball_moving_main_cycle()
        if self.role == 'dance': self.dance_main_cycle()
        self.motion.sim_Progress(1)
        self.motion.sim_Stop()
        self.motion.print_Diagnostics()
        self.motion.sim_Disable()

    def real(self, initial_coord, button):
        self.motion = Motion(self.glob, self.vision, button)
        #print('startup time =', self.clock.avg() )
        pressed_button = self.motion.push_Button(button)
        #pyb.delay(2000)
        self.common_init(initial_coord)
        #eval('self.' + self.role + '_main_cycle()')
        if self.role == 'goalkeeper': self.goalkeeper_main_cycle()
        if self.role == 'penalty_Goalkeeper': self.penalty_Goalkeeper_main_cycle()
        if self.role == 'side_to_side': self.side_to_side_main_cycle()
        if self.role == 'dribbling': self.dribbling_main_cycle()
        if self.role == 'forward': self.forward_main_cycle()
        if self.role == 'forward_v2': self.forward_v2_main_cycle()
        if self.role == 'forward1': self.forward1_main_cycle()
        if self.role == 'penalty_Shooter': self.penalty_Shooter_main_cycle()
        if self.role == 'run_test': self.run_test_main_cycle(pressed_button)
        if self.role == 'spot_walk': self.spot_walk_main_cycle(pressed_button)
        if self.role == 'sidestep_test': self.sidestep_test_main_cycle()
        if self.role == 'corner_kick_1': self.corner_kick_1_main_cycle()
        if self.role == 'corner_kick_2': self.corner_kick_2_main_cycle()
        if self.role == 'dribbling': self.dribbling_main_cycle()
        if self.role == 'dance': self.dance_main_cycle()

    def common_init(self, initial_coord):
        self.glob.pf_coord = initial_coord
        self.motion.direction_To_Attack = -initial_coord[2]
        self.motion.activation()
        #self.motion.direction_To_Attack += math.pi/2
        self.local = Local(self.motion,self.glob, self.vision, coord_odometry = initial_coord)    # [-0.9, 1.3, -math.pi/2]
        self.motion.local = self.local
        self.local.coordinate_record(odometry = True)
        self.g = GoalKeeper(self.motion, self.local, self.glob)
        #self.f = Forward(self.motion, self.local, self.glob)
        self.f = Forward_Vector_Matrix(self.motion, self.local, self.glob)
        self.motion.falling_Flag = 0

    def rotation_test_main_cycle(self, pressed_button):
        number_Of_Cycles = 20
        stepLength = 0
        sideLength = 0
        if pressed_button == 6: rotation = 0.23 # 0.483
        else: rotation = -0.23
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        self.motion.walk_Initial_Pose()
        for cycle in range(number_Of_Cycles):
            self.motion.walk_Cycle(stepLength,sideLength, rotation, cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()
        self.motion.refresh_Orientation()
        print('self.motion.imu_body_yaw() =', self.motion.imu_body_yaw())

    def spot_walk_main_cycle(self, pressed_button):
        if pressed_button == 2 or pressed_button ==3 :
            self.rotation_test_main_cycle(pressed_button)
            return
        self.run_test_main_cycle(1, stepLength = 0)

    def run_test_main_cycle(self, pressed_button, stepLength = 64):
        if pressed_button == 2 or pressed_button ==3 :
            self.sidestep_test_main_cycle(pressed_button)
            return
        if pressed_button == 5 or pressed_button ==6 :
            self.rotation_test_main_cycle(pressed_button)
            return
        stepLength = 64
        if pressed_button == 9: stepLength = 0
        number_Of_Cycles = 20
        if pressed_button == 1: number_Of_Cycles = 100
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if cycle ==0 : stepLength1 = stepLength/3
            if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            rotation = 0 + invert * self.motion.imu_body_yaw() * 1.2
            if rotation > 0: rotation *= 1.5
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()

    def sidestep_test_main_cycle(self, pressed_button):
        number_Of_Cycles = 20
        stepLength = 0 #64
        sideLength = 20
        if pressed_button == 3:
            self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        self.motion.walk_Initial_Pose()
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            #if cycle ==0 : stepLength1 = stepLength/3
            #if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            rotation = 0 + invert * self.motion.imu_body_yaw() * 1.0
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()

    def norm_yaw(self, yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:  yaw -= 2* math.pi
        if yaw < -math.pi: yaw += 2* math.pi
        return yaw

    def forward_main_cycle(self):
        numteek = 0
        first_shoot = False
        while (True):
            print("number of teek: ", numteek)
            numteek +=1
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.call_Par_Filter.pf.fall_reset()
            #while(True):
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True, with_Localization=True)
             #   print("success: ", success_Code)
            print("dist: ", dist)
            print("naprv: ", napravl)
            self.f.dir_To_Guest()
            print('direction_To_Guest = ', math.degrees(self.f.dir_To_Guest()), 'degrees')
            
            print("====="*10)
            print("LOCALIZATION")
            print("Ball: ", self.glob.ball_coord)
            print("Self coord: ", self.glob.pf_coord)
            print("====="*10)
            if dist == 0 and success_Code == False:
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)
                continue
            if dist > 0.35  or self.glob.pf_coord[0] - 0.2 > self.glob.ball_coord[0] :
                if dist > 1: stop_Over = True
                else: stop_Over = False
                print("glob coords: ", self.glob.ball_coord)
                self.motion.far_distance_plan_approach(self.glob.ball_coord, self.f.direction_To_Guest, stop_Over = stop_Over)
                #self.f.turn_Face_To_Guest()
                continue
            #if first_shoot == False:
            if self.motion.params['DRIBBLING'] == 1:
                self.motion.turn_To_Course(self.f.direction_To_Guest)
                success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
                if napravl > 0.2 : 
                    self.motion.near_distance_omni_motion(dist* math.cos(napravl)*1000/2, math.pi/2)
                if napravl < -0.2 : 
                    self.motion.near_distance_omni_motion(dist* math.cos(napravl)*1000, -math.pi/2)
                number_Of_Cycles = 15
                stepLength = 20
                sideLength = 0
                old_neck_pan, old_neck_tilt = self.motion.head_Up()
                self.motion.walk_Initial_Pose()
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    #if cycle ==0 : stepLength1 = stepLength/4
                    #if cycle ==1 : stepLength1 = stepLength/2
                    self.motion.refresh_Orientation()
                    rotation = self.f.direction_To_Guest - self.motion.imu_body_yaw()
                    rotation = self.motion.normalize_rotation(rotation)
                    self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
                self.motion.walk_Final_Pose()
                self.motion.head_Return(old_neck_pan, old_neck_tilt)
            else:
                self.motion.turn_To_Course(self.f.direction_To_Guest)
                success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)
                #motion.near_distance_omni_motion(500, 0)
                #first_shoot = True
            #if self.motion.falling_Flag != 0: continue
            #else:
            #    self.motion.turn_To_Course(self.f.direction_To_Guest)
            #    success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)

    def penalty_Shooter_main_cycle(self):
        first_shoot = True
        while (True):
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.call_Par_Filter.pf.fall_reset()
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
            print('ball_coord = ', self.glob.ball_coord)
            print('direction_To_Guest = ', math.degrees(self.f.dir_To_Guest()), 'degrees')
            if dist == 0 and success_Code == False:
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)
                continue
            if dist > 0.35 or self.glob.ball_coord[0] < self.glob.pf_coord[0] + 0.2:
                if dist > 1: stop_Over = True
                else: stop_Over = False
                self.motion.far_distance_plan_approach(self.glob.ball_coord, self.f.direction_To_Guest, stop_Over = stop_Over)
            kick_direction = self.f.direction_To_Guest
            self.motion.turn_To_Course(kick_direction)
            #if first_shoot:
            if self.motion.params['DRIBBLING'] == 1:
                success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
                if napravl > 0.2 : 
                    self.motion.near_distance_omni_motion(dist* math.cos(napravl)*1000/2, math.pi/2)
                if napravl < -0.2 : 
                    self.motion.near_distance_omni_motion(dist* math.cos(napravl)*1000, -math.pi/2)
                number_Of_Cycles = 15
                stepLength = 20
                sideLength = 0
                old_neck_pan, old_neck_tilt = self.motion.head_Up()
                self.motion.walk_Initial_Pose()
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    #if cycle ==0 : stepLength1 = stepLength/4
                    #if cycle ==1 : stepLength1 = stepLength/2
                    self.motion.refresh_Orientation()
                    rotation = self.f.direction_To_Guest - self.motion.imu_body_yaw()
                    rotation = self.motion.normalize_rotation(rotation)
                    self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
                self.motion.walk_Final_Pose()
                self.motion.head_Return(old_neck_pan, old_neck_tilt)
            else:
                success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)
            #self.motion.near_distance_omni_motion(700, 0)
            first_shoot = False
            #return
            if self.motion.falling_Flag != 0: continue
            #else:
            #    success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)

    def goalkeeper_main_cycle(self):
        self.motion.near_distance_omni_motion(200, 0)                    # get out from goal
        while (True):
            dist = -1.0
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.call_Par_Filter.pf.fall_reset()
                self.g.turn_Face_To_Guest()
                #goto_Center()
            while(dist < 0):
                a, dist,napravl, speed = self.g.find_Ball()
                #uprint('speed = ', speed, 'dist  =', dist , 'napravl =', napravl)
                if abs(speed[0]) > 0.02 and dist < 1 :                         # if dangerous tangential speed
                    if speed[0] > 0:
                        if self.glob.pf_coord[1] < 0.35:
                            self.motion.play_Motion_Slot(name ='PenaltyDefenceL')
                    else:
                        if self.glob.pf_coord[1] > -0.35:
                            self.motion.play_Motion_Slot(name ='PenaltyDefenceR')
                    if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5: time.sleep(3)
                    else: self.motion.sim_Progress(3)
                    continue
                if speed[1] < - 0.01 and dist < 1.5 :                          # if dangerous front speed
                    self.motion.play_Motion_Slot(name = 'PanaltyDefenceReady_Fast')
                    self.motion.play_Motion_Slot(name = 'PenaltyDefenceF')
                    if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5: time.sleep(8)
                    else: self.motion.sim_Progress(3)
                    self.motion.play_Motion_Slot(name = 'Get_Up_From_Defence')

                if (dist == 0 and napravl == 0) or dist > 2.5:
                    position_limit_x1 = -self.glob.landmarks['FIELD_LENGTH']/2 - 0.05
                    position_limit_x2 = position_limit_x1 + 0.25
                    if position_limit_x1 < self.glob.pf_coord[0] < position_limit_x2 and -0.05 < self.glob.pf_coord[1] < 0.05: break
                    self.g.goto_Center()
                    break
                old_neck_pan, old_neck_tilt = self.motion.head_Up()
                if (dist <= 0.7         and 0 <= napravl <= math.pi/4):         self.g.scenario_A1( dist, napravl)
                if (dist <= 0.7         and math.pi/4 < napravl <= math.pi/2):  self.g.scenario_A2( dist, napravl)
                if (dist <= 0.7         and 0 >= napravl >= -math.pi/4):        self.g.scenario_A3( dist, napravl)
                if (dist <= 0.7         and -math.pi/4 > napravl >= -math.pi/2): self.g.scenario_A4( dist, napravl)
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/18 <= napravl <= math.pi/4)): self.g.scenario_B1()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/4 < napravl <= math.pi/2)): self.g.scenario_B2()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/18 >= napravl >= -math.pi/4)): self.g.scenario_B3()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/4 > napravl >= -math.pi/2)): self.g.scenario_B4()
                self.motion.head_Return(old_neck_pan, old_neck_tilt)

    def penalty_Goalkeeper_main_cycle(self):
        self.glob.obstacleAvoidanceIsOn = False
        first_Get_Up = True
        while (True):
            dist = -1.0
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.call_Par_Filter.pf.fall_reset()
                self.g.turn_Face_To_Guest()
                if first_Get_Up:
                    first_Get_Up = False
                    self.g.goto_Center()
            while(dist < 0):
                a, napravl, dist, speed = self.motion.watch_Ball_In_Pose(penalty_Goalkeeper = True)
                #uprint('speed = ', speed, 'dist  =', dist , 'napravl =', napravl)
                if abs(speed[0]) > 0.002 and dist < 1 :                         # if dangerous tangential speed
                    if speed[0] > 0:
                        self.motion.play_Motion_Slot(name ='PenaltyDefenceL')
                    else:
                        self.motion.play_Motion_Slot(name ='PenaltyDefenceR')
                    #if self.glob.SIMULATION == 2: pyb.delay(5000)
                    #else: self.motion.sim_Progress(5)
                    continue
                if speed[1] < - 0.01 and dist < 1.5 :                          # if dangerous front speed
                    self.motion.play_Motion_Slot(name = 'PanaltyDefenceReady_Fast')
                    self.motion.play_Motion_Slot(name = 'PenaltyDefenceF')
                    if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5: time.sleep(5)
                    else: self.motion.sim_Progress(5)
                    self.motion.play_Motion_Slot(name = 'Get_Up_From_Defence')

                if (dist == 0 and napravl == 0) or dist > 2.5:
                    #position_limit_x1 = -self.glob.landmarks['FIELD_LENGTH']/2 - 0.05
                    #position_limit_x2 = position_limit_x1 + 0.25
                    #if position_limit_x1 < self.glob.pf_coord[0] < position_limit_x2 and -0.05 < self.glob.pf_coord[1] < 0.05: break
                    #self.g.goto_Center()
                    #break
                    continue
                old_neck_pan, old_neck_tilt = self.motion.head_Up()
                if (dist <= 0.7         and 0 <= napravl <= math.pi/4):         self.g.scenario_A1( dist, napravl)
                if (dist <= 0.7         and math.pi/4 < napravl <= math.pi/2):  self.g.scenario_A2( dist, napravl)
                if (dist <= 0.7         and 0 >= napravl >= -math.pi/4):        self.g.scenario_A3( dist, napravl)
                if (dist <= 0.7         and -math.pi/4 > napravl >= -math.pi/2): self.g.scenario_A4( dist, napravl)
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/18 <= napravl <= math.pi/4)): self.g.scenario_B1()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/4 < napravl <= math.pi/2)): self.g.scenario_B2()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/18 >= napravl >= -math.pi/4)): self.g.scenario_B3()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/4 > napravl >= -math.pi/2)): self.g.scenario_B4()
                self.motion.head_Return(old_neck_pan, old_neck_tilt)

    def dance_main_cycle(self):
        if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
            while True:
                successCode, u10 = self.motion.kondo.getUserParameter(10)
                #time.sleep(2)
                if successCode and u10 == 1:
                    self.motion.kondo.motionPlay(26)
                    for i in range(10):
                        self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
                    self.motion.play_Soft_Motion_Slot( name = 'Dance_7')
                    for i in range(9):
                        self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
                    self.motion.play_Soft_Motion_Slot( name = 'Dance_2')
                    for i in range(2):
                        self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
                    self.motion.play_Soft_Motion_Slot( name = 'Dance_4')
                    self.motion.kondo.setUserParameter(10,0)
        else:
            for i in range(10):
                self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_7')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_2')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_4')


if __name__=="__main__":
    pass