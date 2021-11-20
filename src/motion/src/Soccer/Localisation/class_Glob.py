import json, array, math

class Glob:
    def __init__(self, simulation, current_work_directory, particles_number = 1000):
        self.COLUMNS = 18
        self.ROWS = 13
        self.current_work_directory = current_work_directory
        self.particles_number = particles_number
        self.pf_alloc1 = array.array('I',(0 for i in range(particles_number*4)))
        self.pf_alloc2 = array.array('I',(0 for i in range(particles_number*4)))
        self.weights = array.array('I',(0 for i in range(particles_number)))
        self.new_p = array.array('I',(0 for i in range(particles_number)))
        self.strategy_data = array.array('b',(0 for i in range(self.COLUMNS * self.ROWS * 2)))
        self.SIMULATION = simulation             # 0 - Simulation without physics, 1 - Simulation with physics, 2 - live on openMV
        self.ball_coord =[0.0,0.0]
        self.pf_coord = [0.0,0.0,0.0]
        self.obstacles = []
        if self.SIMULATION == 1 or self.SIMULATION == 0 or self.SIMULATION == 3:
            self.SIM_OPTION = 'SURROGAT'  # other variants: 'KONDOv23',  'ODE'
            with open(current_work_directory + "Init_params/Sim/Sim_landmarks.json", "r") as f:
                landmarks = json.loads(f.read())
            with open(current_work_directory + "Init_params/Sim/" + self.SIM_OPTION + "/Sim_params.json", "r") as f:
                self.params = json.loads(f.read())
        elif self.SIMULATION == 2 or self.SIMULATION == 5:
            with open(current_work_directory + "Init_params/Real/Real_landmarks.json", "r") as f:
                landmarks = json.loads(f.read())
            with open(current_work_directory + "Init_params/Real/Real_params.json", "r") as f:
                self.params = json.loads(f.read())
        self.first_step_yield = (19 * self.params['RUN_TEST_10_STEPS'] - 9 * self.params['RUN_TEST_20_STEPS']) / 10
        self.cycle_step_yield = ( self.params['RUN_TEST_20_STEPS'] - self.params['RUN_TEST_10_STEPS']) / 10
        self.side_step_right_yield = self.params['SIDE_STEP_RIGHT_TEST_RESULT'] / 20
        self.side_step_left_yield = self.params['SIDE_STEP_LEFT_TEST_RESULT'] / 20
        #print("self.first_step_yield", self.first_step_yield)
        #print("self.cycle_step_yield", self.cycle_step_yield)
        #print("self.side_step_right_yield", self.side_step_right_yield)
        #print(self.params)
        self.landmarks = landmarks
        self.import_strategy_data(current_work_directory)
        self.obstacleAvoidanceIsOn = False

    def import_strategy_data(self, current_work_directory):
        with open(current_work_directory + "Init_params/strategy_data.json", "r") as f:
            loaded_Dict = json.loads(f.read())
        if loaded_Dict.get('strategy_data') != None:
            strategy_data = loaded_Dict['strategy_data']
        for column in range(self.COLUMNS):
            for row in range(self.ROWS):
                index1 = column * self.ROWS + row
                power = strategy_data[index1][2]
                yaw = int(strategy_data[index1][3] * 40)  # yaw in radians multiplied by 40
                self.strategy_data[index1*2] = power
                self.strategy_data[index1*2+1] = yaw



