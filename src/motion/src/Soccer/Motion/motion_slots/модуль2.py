import class_Motion as mt
from ball_Approach_Steps_Seq import *
import math, random
import class_Local as lc
import sys, json




def uprint(*text):
    with open("output.txt",'a') as f:
        print(*text, file = f)
        print(*text )
class GoalKeeper:
    def __init__(self, simulation, motion, local):
        self.motion = motion
        self.local = local
        if simulation == 2: 
            with open("live_params.json", "r") as f:
                params = json.loads(f.read())
        else: 
            with open("C:/Users/a/Documents/VREP/remoteApiBindings/python/python/sim_params.json", "r") as f:
                params = json.loads(f.read())
        self.side_step_yield = params['SIDE_STEP_YIELD']
        self.first_step_yield =  params['FIRST_STEP_YIELD']
        self.cycle_step_yield =  params['CYCLE_STEP_YIELD']


    def near_distance_ball_approach_and_kick(self):
        self.motion.turn_To_Course(self.motion.direction_To_Attack)
        self.local.post_data_in_pose = []
        self.local.quality = 0
        a, napravl, dist_mm = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
        self.local.localisation_Complete()
        if a==False and self.motion.falling_Flag != 0: return False
        if dist_mm > 900 or a == False: return False
        if  20 < abs(dist_mm*math.cos(math.radians(napravl))) < 90 and dist_mm*math.sin(math.radians(napravl)) < 50:
            self.motion.head_Up()
            if napravl > 0: self.motion.kick(first_Leg_Is_Right_Leg=False)
            else: self.motion.kick(first_Leg_Is_Right_Leg=True)
            self.motion.head_Return()
        if abs(napravl) > 60 :
            self.motion.head_Up()
            self.motion.walk_Initial_Pose()
            self.motion.walk_Cycle(stepLength = -15, sideLength = - math.copysign(20, napravl),rotation = 0,cycle = 0, number_Of_Cycles = 3)
            self.motion.walk_Cycle(stepLength = -30, sideLength = - math.copysign(20, napravl),rotation = 0,cycle = 1, number_Of_Cycles = 3)
            self.motion.walk_Cycle(stepLength = -60, sideLength = - math.copysign(20, napravl),rotation = 0,cycle = 2, number_Of_Cycles = 3)
            self.motion.walk_Final_Pose()
            compo_step = self.first_step_yield*15/64 + self.cycle_step_yield*30/64 + self.cycle_step_yield*60/64
            self.local.coord_odometry[0] +=int(-compo_step*math.cos(math.radians(self.motion.euler_angle[0] - self.motion.direction_To_Attack + self.motion.neck_pan*0.03375)))
            self.local.coord_odometry[1] +=int(-compo_step*math.sin(math.radians(self.motion.euler_angle[0] - self.motion.direction_To_Attack + self.motion.neck_pan*0.03375)))
            self.local.coordinate_record(odometry = True)
            self.motion.head_Return()

        else:
            n = int(math.floor((dist_mm-100-self.first_step_yield)/self.cycle_step_yield)+1)+1         #calculating the number of potential full steps forward
            displacement = dist_mm*math.sin(math.radians(napravl))
            m = int(math.floor(abs(abs(displacement)-53.4)/self.side_step_yield)+1)
            if n < m : n = m
            stepLength = (dist_mm-100)/(self.first_step_yield*1.25+self.cycle_step_yield*(n-1)+ self.cycle_step_yield*0.75)*64
            number_Of_Cycles = n+2
            if napravl > 0:
                sideLength = -(displacement - 53.4)/number_Of_Cycles*20/self.side_step_yield
                kick_by_Right = False
            else:
                sideLength = -(displacement + 53.4)/number_Of_Cycles*20/self.side_step_yield
                kick_by_Right = True
            self.motion.head_Up()
            self.motion.walk_Initial_Pose()
            for cycle in range(number_Of_Cycles):
                rotation = self.motion.direction_To_Attack-(self.motion.neck_pan*0.03375 + self.motion.euler_angle[0])*1
                if rotation > 30 : rotation = 30
                if rotation < -30 : rotation = -30
                stepLength1 = stepLength
                if cycle == 0: stepLength1 = stepLength/4
                if cycle == 1: stepLength1 = stepLength/2
                self.motion.walk_Cycle(stepLength1, sideLength,rotation,cycle,number_Of_Cycles)
            self.motion.walk_Final_Pose()
            self.motion.kick(first_Leg_Is_Right_Leg=kick_by_Right)
            self.local.coord_odometry[0] += int(dist_mm*math.cos(math.radians(napravl)))
            self.local.coord_odometry[1] += int(dist_mm*math.sin(math.radians(napravl)))
            self.local.coordinate_record(odometry = True)
            self.motion.head_Return()

        return True


    def goto_Center(self):#Function for reterning to center position
        uprint('Function for reterning to center position')
        #x1, y1, z1 = self.motion.Dummy_HData[len(self.motion.Dummy_HData)-1]        # reading position from simulation service
        if self.local.quality < 1: self.motion.localisation_Motion()
        player_X_m = self.local.coordinate[0]/1000
        player_Y_m = self.local.coordinate[1]/1000
        ball_X_m, ball_Y_m = -1.6, 0
        self.motion.head_Up()
        ball_Approach(self.motion, self.local, ball_X_m, ball_Y_m, player_X_m, player_Y_m)
        self.motion.head_Return()

    def find_Ball(self):
        #dist = float(input('dist ='))
        #napravl = float(input('napravl ='))
        a, napravl, distance = self.motion.seek_Ball_In_Pose(fast_Reaction_On=True)
        self.local.localisation_Complete()
        dist = distance/1000
        uprint ( 'dist = ', dist, 'napravl =', napravl)
        return dist, napravl
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
                player_X_m, player_Y_m = self.local.coordinate[0]/1000 , self.local.coordinate[1]/1000
                napravl_rad = math.radians(napravl)
                ball_X_m = dist* math.cos(napravl_rad) + player_X_m
                ball_Y_m = dist* math.sin(napravl_rad) + player_Y_m
                ball_Approach(self.motion, self.local,  ball_X_m, ball_Y_m, player_X_m, player_Y_m)
                a = self.near_distance_ball_approach_and_kick()
            else: a = self.near_distance_ball_approach_and_kick()
            if a==False and self.motion.falling_Flag != 0: return
            if a == False : break
            a, napravl, distance = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
            self.local.localisation_Complete()
            dist = distance / 1000
            if dist > 1 : break 
        target_course1 = self.motion.euler_angle[0] +180
        self.motion.turn_To_Course(target_course1)
        a, napravl, distance = self.motion.seek_Ball_In_Pose(fast_Reaction_On = False)
        self.local.localisation_Complete()
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
        self.motion.first_Leg_Is_Right_Leg = True
        self.motion.walk_Initial_Pose()
        stepLength, sideLength, rotation, cycle, number_Of_Cycles = 0, -20 , 0 , 0 , 4
        for cycle in range(number_Of_Cycles): self.motion.walk_Cycle(stepLength, sideLength,rotation,cycle,number_Of_Cycles)
        self.local.coord_odometry[0] += int(-self.side_step_yield * 4 * math.sin( math.radians(self.motion.euler_angle[0] - self.motion.direction_To_Attack + self.motion.neck_pan*0.03375)))
        self.local.coord_odometry[1] += int(self.side_step_yield * 4 * math.cos( math.radians(self.motion.euler_angle[0] - self.motion.direction_To_Attack + self.motion.neck_pan*0.03375)))
        self.local.coordinate_record(odometry = True)
        self.motion.walk_Final_Pose()
        self.motion.turn_To_Course(self.motion.direction_To_Attack)

    def scenario_B2(self):#the robot moves to the left and stands on the same axis as the ball and the opponents' goal
        uprint('the robot moves to the left 4 steps')
        self.scenario_B1()

    def scenario_B3(self):#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
        uprint('the robot moves to the right 4 steps')
        self.motion.first_Leg_Is_Right_Leg = True
        self.motion.walk_Initial_Pose()
        stepLength, sideLength, rotation, cycle, number_Of_Cycles = 0, 20 , 0 , 0 , 4
        for cycle in range(number_Of_Cycles): self.motion.walk_Cycle(stepLength, sideLength,rotation,cycle,number_Of_Cycles)
        self.motion.walk_Final_Pose()
        self.local.coord_odometry[0] += int(self.side_step_yield * 4 * math.sin( math.radians(self.motion.euler_angle[0] - self.motion.direction_To_Attack + self.motion.neck_pan*0.03375)))
        self.local.coord_odometry[1] += int(-self.side_step_yield * 4 * math.cos( math.radians(self.motion.euler_angle[0] - self.motion.direction_To_Attack + self.motion.neck_pan*0.03375)))
        self.local.coordinate_record(odometry = True)
        self.motion.turn_To_Course(self.motion.direction_To_Attack)

    def scenario_B4(self):#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
        uprint('the robot moves to the right 4 steps')
        self.scenario_B3()

def main():
    SIMULATION = 1
    motion = mt.Motion(SIMULATION)
    local = lc.Local(motion)
    motion.local = local
    g = GoalKeeper(SIMULATION, motion, local)
    if SIMULATION == 2:
        import pyb
        motion.push_Button()
        pyb.delay(2000)
    else:
        motion.sim_Enable()
        motion.Vision_Sensor_Display_On = False
    motion.sim_Start()
    local.coord_odometry = [-1800, 0, 0]
    local.coordinate_record(odometry = True)

    motion.falling_Flag = 0
    while (True):
        Ballposition = motion.Ballposition
        #Ballposition[0] = random.uniform(-1.7, 0)
        #Ballposition[1] = random.uniform(-1.3, 1.3)
        if input('continue? (y/n)') == 'n': break
        #returnCode = motion.vrep.simxSetObjectPosition(motion.clientID, motion.BallHandle , motion.ResizableFloorHandle,Ballposition, motion.vrep.simx_opmode_oneshot)
        dist = -1.0
        if motion.falling_Flag != 0:
            motion.falling_Flag = 0
            motion.turn_To_Course(motion.direction_To_Attack)
            #goto_Center()
        while(dist < 0):
            dist,napravl = g.find_Ball()
            print(Ballpisition, local.ball_coord)
            #if dist == 0 and napravl == 0:
            #    if -1850 < local.coord_odometry[0] < -1700 and -50 < local.coord_odometry[1] < 50: break
            #    g.goto_Center()
            #    break
            #if g.ball_Speed_Dangerous():
            #    g.fall_to_Defence()
            #    time.sleep(3.0)
            #    g.get_Up_from_defence()
            #motion.head_Up()
            #if (dist <= 0.9 and 0 <= napravl <= 45): g.scenario_A1( dist, napravl)
            #if (dist <= 0.9 and 45 < napravl <= 90): g.scenario_A2( dist, napravl)
            #if (dist <= 0.9 and 0 >= napravl >= -45): g.scenario_A3( dist, napravl)
            #if (dist <= 0.9 and -45 > napravl >= -90): g.scenario_A4( dist, napravl)
            #if ((1.8 >= dist > 0.9) and (10 <= napravl <= 45)): g.scenario_B1()
            #if ((1.8 >= dist > 0.9) and (45 < napravl <= 90)): g.scenario_B2()
            #if ((1.8 >= dist > 0.9) and (-10 >= napravl >= -45)): g.scenario_B3()
            #if ((1.8 >= dist > 0.9) and (-45 > napravl >= -90)): g.scenario_B4()
            #motion.head_Return()


    motion.sim_Progress(2)
    motion.sim_Stop()
    #motion.print_Diagnostics()
    motion.sim_Disable()




if __name__=="__main__":
    #try:
    main()

    #except Exception as e:
        #with open("output.txt",'a') as f:
        #    sys.print_exception(e,f)
        #    sys.print_exception(e,sys.stdout)



