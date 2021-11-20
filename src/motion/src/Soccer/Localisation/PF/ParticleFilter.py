
import sys, os, json, array

#current_work_directory = os.getcwd()
#current_work_directory = current_work_directory.replace('\\', '/')
if sys.version == '3.8.5 (default, Jan 27 2021, 15:41:15) \n[GCC 9.3.0]':
    #current_work_directory += '/'
    from random import *
    #import numpy as np
    used_with_OpenMV = False
else:
    print('!')
    import time
    from urandom import getrandbits
    #import ulab as np
    #import starkit
    used_with_OpenMV = False

used_with_OpenMV_firmware = False


#sys.path.append( current_work_directory + 'Soccer/')
#sys.path.append( current_work_directory + 'Soccer/Motion/')
#sys.path.append( current_work_directory + 'Soccer/Vision/')
#sys.path.append( current_work_directory + 'Soccer/Localisation/')
#sys.path.append( current_work_directory + 'Soccer/Localisation/PF/')


import math
import json
import time

SDVIG = 32768


def weight_calculation( n, weights, observations, landmarks, gauss_noise, line_gauss_noise, p):

    def gaussian( x, sigma):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return math.exp(-(x ** 2) / 2*(sigma ** 2)) / math.sqrt(2.0 * math.pi * (sigma ** 2))

    def new_observation_score( i, observations, landmarks, gauss_noise, p):
        # particle weight calculation
        prob = 1.0
        part0 = (p[i * 4] - SDVIG)/ 1000
        part1 = (p[i * 4 + 1] - SDVIG)/ 1000
        part2 = (p[i * 4 + 2] - SDVIG)/ 1000
        sin_p = math.sin(part2) 
        cos_p = math.cos(part2)
        for color_landmarks in observations:
            if (color_landmarks not in landmarks):
                continue
            if (color_landmarks == 'lines'):
                continue
            for landmark in landmarks[color_landmarks]:
                min_dist = 1000
                if observations[color_landmarks]:
                    for observation in observations[color_landmarks]:
                        # calc posts coords in field for every mesurement
                        x_posts = part0 + observation[0] * cos_p - observation[1] * sin_p
                        y_posts = part1 + observation[0] * sin_p + observation[1] * cos_p
                        dist =(x_posts - landmark[0])**2 + (y_posts - landmark[1])**2
                        if min_dist > dist:
                            min_dist = dist
                            weight = observation[2]
                if min_dist != 1000:
                    prob *= gaussian(math.sqrt(min_dist), gauss_noise)*weight
        return prob
    def new_calc_lines_score( i, lines, landmarks, line_gauss_noise, p):
        '''
        line = (ro, theta)
        '''
        part0 = (p[i * 4] - SDVIG)/ 1000
        part1 = (p[i * 4 + 1] - SDVIG)/ 1000
        part2 = (p[i * 4 + 2] - SDVIG)/ 1000
        prob = 1
        if lines != []:
            for line in lines:
                min_dist = 1000
                for landmark_line in landmarks:
                    for coord in landmarks[landmark_line]:
                        yaw = (part2 + line[1])%(2*math.pi)
                        if landmark_line == 'x':
                            dist = math.fabs(coord - (part0 + line[0]*math.cos(yaw)))
                        else:
                            dist = math.fabs(coord - (part1 + line[0]*math.sin(yaw)))
                        if min_dist > dist:
                            min_dist = dist
                if min_dist != 1000:
                    prob *= gaussian(min_dist, line_gauss_noise)*line[2]
        return prob
    # тело функции
    S = 0.0
    for i in range(n):
        weight = int(new_observation_score(i,observations, landmarks, gauss_noise, p)
                            *new_calc_lines_score(i,observations['lines'], landmarks['lines'], line_gauss_noise, p)
                            * 20000)
        weights[i] = weight
        S += weight
    return weights, S

def randrange( start, stop=None):
#helper function for working with random bit sequence
    if stop is None:
        stop = start
        start = 0
    upper = stop - start
    bits = 0
    pwr2 = 1
    while upper > pwr2:
        pwr2 <<= 1
        bits += 1
    while True:
        r = getrandbits(bits)
        if r < upper:
            break
    return r + start

def random():
    #getting a random number from 0 to 1
    return randrange(10000) / 10000

def gauss( mu, sigma):
    #getting a random number from Gaussian distribution
    x2pi = random() * math.pi * 2
    g2rad = math.sqrt(-2.0 * math.log(1.0 - random()))
    z = math.cos(x2pi) * g2rad
    return mu + z * sigma

def gaussian( x, sigma):
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return math.exp(-(x ** 2) / 2*(sigma ** 2)) / math.sqrt(2.0 * math.pi * (sigma ** 2))


class ParticleFilter():
    def __init__(self, myrobot, glob):
        self.counter1 = 0
        self.counter2 = 0
        self.n = glob.particles_number
        self.myrobot = myrobot
        self.count = 0
        self.p = glob.pf_alloc1
        self.tmp = glob.pf_alloc2
        self.weights = glob.weights
        self.new_p = glob.new_p
        #self.logger = PFlogger()
        #self.viz = RealtimeViz()
        self.landmarks = glob.landmarks
        self.landmark_lines_x = array.array('f',self.landmarks['lines']['x'])
        self.landmark_lines_y = array.array('f',self.landmarks['lines']['y'])
        self.land_keys =list(self.landmarks.keys())
        self.land_keys.pop(self.land_keys.index('lines'))
        self.land_keys.pop(self.land_keys.index('FIELD_WIDTH'))
        self.land_keys.pop(self.land_keys.index('FIELD_LENGTH'))
        self.array_landmarks = []
        for i in range(len(self.land_keys)):
            marks = array.array('f',[])
            for j in range(len(self.landmarks[self.land_keys[i]])):
                mark1 = array.array('f',self.landmarks[self.land_keys[i]][j])
                marks.extend(mark1)
            self.array_landmarks.append(marks)
        with open(glob.current_work_directory + "Soccer/Localisation/PF/pf_constants.json", 'r') as constants:
            constants = json.load(constants)
        self.forward_noise = constants['noise']['forward_noise']
        self.turn_noise = constants['noise']['turn_noise']
        self.sense_noise = constants['noise']['sense_noise']
        self.gauss_noise = constants['noise']['gauss_noise']
        self.yaw_noise = constants['noise']['yaw_noise']
        self.line_gauss_noise = constants['noise']['line_gauss_noise']
        self.other_coord_noise = constants['noise']['other_coord_noise']

        self.number_of_res = constants['consistency']['number_of_res']
        self.consistency = constants['consistency']['consistency']
        self.goodObsGain = constants['consistency']['goodObsGain']
        self.badObsCost = constants['consistency']['badObsCost']
        self.stepCost = constants['consistency']['stepCost']
        self.dist_threshold = constants['consistency']['dist_threshold']
        self.con_threshold = constants['consistency']['con_threshold']
        self.spec_threshold = constants['consistency']['spec_threshold']

        #self.token = ''
        #self.logs = open(current_work_directory + "Soccer/Localisation/PF/logs/logs" + self.token + '.txt', "w") #
        #self.obs_logs = open(current_work_directory + "Soccer/Localisation/PF/logs/observations.json", "w") #
        #self.od_logs = open(current_work_directory + "Soccer/Localisation/PF/logs/odometry.json", "w") #
        #self.other_logs = open(current_work_directory + "Soccer/Localisation/PF/logs/other_coord.json", "w")
        #self.od_logs.close()
        #self.obs_logs.close()
        #self.other_logs.close()
        self.gen_particles()

    def return_coord(self):
        return self.myrobot.x, self.myrobot.y, self.myrobot.yaw

    def gen_particles(self):
        for i in range(self.n):
            x_coord = self.myrobot.x + gauss(0, self.sense_noise)
            y_coord = self.myrobot.y + gauss(0, self.sense_noise)
            yaw = self.myrobot.yaw + gauss(0, self.yaw_noise)*math.pi
            if yaw < 0:
                yaw = 2*math.pi + yaw
            if yaw > 2*math.pi:
                yaw %= (2 * math.pi)
            #self.byte_put(self.p, i * 4 + 0, int(x_coord * 1000) + SDVIG)
            #self.byte_put(self.p, i * 4 + 1, int(y_coord * 1000) + SDVIG)
            #self.byte_put(self.p, i * 4 + 2, int(yaw * 1000) + SDVIG)
            #self.p[i, 0] = int(x_coord * 1000) + SDVIG
            #self.p[i, 1] = int(y_coord * 1000) + SDVIG
            #self.p[i, 2] = int(yaw * 1000) + SDVIG
            self.p[i * 4 + 0] = int(x_coord * 1000) + SDVIG
            self.p[i * 4 + 1] = int(y_coord * 1000) + SDVIG
            self.p[i * 4 + 2] = int(yaw * 1000) + SDVIG
        self.limit_paricles_coord(self.p, self.n)
        #self.logger.step("initial,step",self.return_coord(), self.p)
        self.count += 1

    def gen_n_particles_robot(self, start_row):
        for i in range(start_row, self.n, 1):
            x_coord = self.myrobot.x + gauss(0, self.sense_noise*3)
            y_coord = self.myrobot.y + gauss(0, self.sense_noise*3)
            yaw = self.myrobot.yaw + gauss(0, self.yaw_noise)*math.pi
            if yaw < 0:
                yaw = 2*math.pi + yaw
            if yaw > 2*math.pi:
                yaw %= (2 * math.pi)
            #self.byte_put(self.p, i * 4 + 0, int(x_coord * 1000) + SDVIG)
            #self.byte_put(self.p, i * 4 + 1, int(y_coord * 1000) + SDVIG)
            #self.byte_put(self.p, i * 4 + 2, int(yaw * 1000) + SDVIG)
            #self.p[i, 0] = int(x_coord * 1000) + SDVIG
            #self.p[i, 1] = int(y_coord * 1000) + SDVIG
            #self.p[i, 2] = int(yaw * 1000) + SDVIG
            self.p[i * 4 + 0] = int(x_coord * 1000) + SDVIG
            self.p[i * 4 + 1] = int(y_coord * 1000) + SDVIG
            self.p[i * 4 + 2] = int(yaw * 1000) + SDVIG


    def gen_n_particles_robot_coord(self, start_row, coord):
        for i in range(start_row, self.n, 1):
            x_coord = coord[0] + gauss(0, self.other_coord_noise)
            y_coord = coord[1] + gauss(0, self.other_coord_noise)
            yaw = coord[2] + gauss(0, self.yaw_noise)*2
            if yaw < 0:
                yaw = 2*math.pi + yaw
            if yaw > 2*math.pi:
                yaw %= (2 * math.pi)
            #self.byte_put(self.p, i * 4 + 0, int(x_coord * 1000) + SDVIG)
            #self.byte_put(self.p, i * 4 + 1, int(y_coord * 1000) + SDVIG)
            #self.byte_put(self.p, i * 4 + 2, int(yaw * 1000) + SDVIG)
            #self.p[i, 0] = int(x_coord * 1000) + SDVIG
            #self.p[i, 1] = int(y_coord * 1000) + SDVIG
            #self.p[i, 2] = int(yaw * 1000) + SDVIG
            self.p[i * 4 + 0] = int(x_coord * 1000) + SDVIG
            self.p[i * 4 + 1] = int(y_coord * 1000) + SDVIG
            self.p[i * 4 + 2] = int(yaw * 1000) + SDVIG

    def update_consistency(self, observations):
        stepConsistency = 0
        for color_landmarks in observations:
            if (color_landmarks not in self.landmarks):
                continue
            if (color_landmarks == 'lines'):
                continue
            if len(observations[color_landmarks]) != 0:
                for observation in observations[color_landmarks]:
                    dists = []
                    for landmark in self.landmarks[color_landmarks]:

                        # calc posts coords in field for every mesurement
                        x_posts = (self.myrobot.x + observation[0]*math.cos(self.myrobot.yaw)
                                   - observation[1]*math.sin(self.myrobot.yaw))
                        y_posts = (self.myrobot.y + observation[0]*math.sin(self.myrobot.yaw)
                                   + observation[1]*math.cos(self.myrobot.yaw))
                        #print('x_posts, y_posts', x_posts, y_posts)
                        dist = math.sqrt(
                            (x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                        dists.append(dist)
                        #print('dist, len =', dist, len(dists))
                    if min(dists) < self.dist_threshold:
                        stepConsistency += self.goodObsGain

                        #print('good step', stepConsistency)
                    else:
                        stepConsistency -= self.badObsCost
                        #print('bad step', stepConsistency)
            else:
                stepConsistency -= self.stepCost
        #print('step cons', stepConsistency)
        self.consistency += stepConsistency
        if self.consistency > self.spec_threshold:
            self.consistency = self.spec_threshold
        elif self.consistency < 0.0:
            self.consistency = 0.0
        print('consistency', self.consistency)

    def particles_move(self, coord):
        #actions = {str(self.count):coord}
        #with open(current_work_directory + "Soccer/Localisation/PF/logs/odometry.json", "a", encoding="utf-8") as self.od_logs:
        #    json.dump(actions, self.od_logs)
        self.myrobot.move(coord['shift_x'],
                          coord['shift_y'], coord['shift_yaw'])
        #print('eto coord after mooving', self.return_coord(),
        # now we simulate a robot motion for each of
        # these particles
        #if used_with_OpenMV:
        #    for i in range(self.n):
        #        orientation = (tuple(self.p[i, 2])[0] - SDVIG)/1000 + float(coord['shift_yaw'])
        #        if orientation < 0:
        #            orientation += (math.pi*2)
        #        orientation %= (2 * math.pi)
        #        self.p[i, 0] = int(tuple(self.p[i, 0])[0]
        #                           + (coord['shift_x'] * math.cos((tuple(self.p[i, 2])[0] - SDVIG)/1000)
        #                           - coord['shift_y'] * math.sin((tuple(self.p[i, 2])[0] - SDVIG)/1000)) * 1000)
        #        self.p[i, 1] = int(tuple(self.p[i, 1])[0]
        #                           + (coord['shift_x'] * math.sin((tuple(self.p[i, 2])[0] - SDVIG)/1000)
        #                           + coord['shift_y'] * math.cos((tuple(self.p[i, 2])[0] - SDVIG)/1000)) * 1000)
        #        self.p[i, 2] = int(orientation * 1000 + SDVIG)
        #else:
        #    for i in range(self.n):
        #        orientation = (self.p[i, 2] - SDVIG) / 1000 + float(coord['shift_yaw'])
        #        if orientation < 0:
        #            orientation += (math.pi*2)
        #        orientation %= (2 * math.pi)
        #        self.p[i, 0] = int(self.p[i, 0]
        #                           + (coord['shift_x'] * math.cos((self.p[i, 2] - SDVIG) / 1000)
        #                           - coord['shift_y'] * math.sin((self.p[i, 2] - SDVIG) / 1000)) * 1000)
        #        self.p[i, 1] = int(self.p[i, 1]
        #                           + (coord['shift_x'] * math.sin((self.p[i, 2] - SDVIG) / 1000)
        #                           + coord['shift_y'] * math.cos((self.p[i, 2] - SDVIG) / 1000)) * 1000)
                #self.p[i, 2] = int(orientation * 1000 + SDVIG)
        for i in range(self.n):
            #orientation = (self.byte_get(self.p, i * 4 + 2) - SDVIG) / 1000 + float(coord['shift_yaw'])
            orientation = (self.p[i * 4 + 2] - SDVIG) / 1000 + float(coord['shift_yaw'])
            if orientation < 0:
                orientation += (math.pi*2)
            orientation %= (2 * math.pi)
            #x = self.byte_get(self.p, i * 4)
            #y = self.byte_get(self.p, i * 4 + 1)
            #yaw = (self.byte_get(self.p, i * 4 + 2) - SDVIG) / 1000
            x = self.p[i * 4]
            y = self.p[i * 4 + 1]
            yaw = (self.p[i * 4 + 2] - SDVIG) / 1000
            x1 = int(x + (coord['shift_x'] * math.cos(yaw) - coord['shift_y'] * math.sin(yaw)) * 1000)
            y1 = int(y + (coord['shift_x'] * math.sin(yaw) + coord['shift_y'] * math.cos(yaw)) * 1000)
            yaw1 = int(orientation * 1000 + SDVIG)
            #self.byte_put(self.p, i * 4, x1)
            #self.byte_put(self.p, i * 4 + 1, y1)
            #self.byte_put(self.p, i * 4 + 2, yaw1)
            self.p[i * 4] = x1
            self.p[i * 4 + 1] = y1
            self.p[i * 4 + 2] = yaw1

        #self.logger.step("moving,step",self.return_coord(), self.p)
        self.count += 1
        self.limit_paricles_coord(self.p, self.n)




    #def fun(x):
    #    #return 0.0634*(x**1.0345)
    #    return 0.1316*math.log(x) + 0.0566
    #    #return (0.0084*(x**3) - 0.0665*(x**2) + 0.2246*x - 0.1071)

    #def additional_weights_culc(self, observations):
    #    for color_landmarks in observations:
    #        if (color_landmarks not in self.landmarks):
    #            continue

    #        if len(observations[color_landmarks]) != 0:
    #            for observation in observations[color_landmarks]:
    #                for j in experimental_data[k][d]:
    #                    if color_landmarks!='lines':
    #                        j[2] = 1 - (fun(math.sqrt((j[0])**2+(j[1])**2))/1.8)
    #                        print(j, 'notline')
    #                    elif color_landmarks=='lines':
    #                        j[2] = 1 - math.fabs(fun(math.fabs(j[1]))/1.8)
    #                        print(j, 'line')
    #    return observations

    def observation_to_predict(self, observations):
        predicts = []
        for color_landmarks in observations:
            if (color_landmarks not in self.landmarks):
                continue
            if ((color_landmarks == 'angle') or (color_landmarks == 'lines')):
                continue
            for landmark in self.landmarks[color_landmarks]:
                if len(observations[color_landmarks]) != 0:
                    for obs in observations[color_landmarks]:
                        y_posts = self.myrobot.x + \
                            obs[0]*math.sin(-self.myrobot.yaw) + \
                            obs[1]*math.cos(-self.myrobot.yaw)
                        x_posts = self.myrobot.y + \
                            obs[0]*math.cos(-self.myrobot.yaw) - \
                            obs[1]*math.sin(-self.myrobot.yaw)
                        predicts.append([x_posts, y_posts])
        return predicts

    def limit_paricles_coord(self, p, stop_row):
        half_w = self.landmarks['FIELD_WIDTH'] / 2
        half_l = self.landmarks['FIELD_LENGTH'] / 2
        #if used_with_OpenMV:
        #    for i in range(stop_row):
        #        x = (tuple(p[i, 0])[0] - SDVIG) / 1000
        #        y = (tuple(p[i, 1])[0] - SDVIG) / 1000
        #        if abs(x) > half_l + 0.3:
        #            p[i, 0] = int(math.copysign(half_l, x)) * 1000 + SDVIG
        #        if math.fabs(y) > half_w + 0.3:
        #            p[i, 1] = int(math.copysign(half_w, y)) * 1000 + SDVIG
        #else:
        #    for i in range(stop_row):
        #        x = (p[i, 0] - SDVIG) / 1000
        #        y = (p[i, 1] - SDVIG) / 1000
        #        if math.fabs(x) > half_l + 0.3:
        #            p[i, 0] = int(math.copysign(half_l, x)) * 1000 + SDVIG
        #        if math.fabs(y) > half_w + 0.3:
        #            p[i, 1] = int(math.copysign(half_w, y)) * 1000 + SDVIG
        #for i in range(stop_row):
        #    x = (self.byte_get(p, i * 4) - SDVIG) / 1000
        #    y = (self.byte_get(p, i * 4 + 1) - SDVIG) / 1000
        #    if math.fabs(x) > half_l + 0.3:
        #        x1 = int(math.copysign(half_l, x)) * 1000 + SDVIG
        #        self.byte_put(p, i * 4, x1)
        #    if math.fabs(y) > half_w + 0.3:
        #        y1 = int(math.copysign(half_w, y)) * 1000 + SDVIG
        #        self.byte_put(p, i * 4 + 1, y1
        for i in range(stop_row):
            x = (p[i * 4] - SDVIG) / 1000
            y = (p[i * 4 + 1] - SDVIG) / 1000
            if math.fabs(x) > half_l + 0.3:
                p[i * 4] = int(math.copysign(half_l + 0.3, x) * 1000 + SDVIG)
            if math.fabs(y) > half_w + 0.3:
                p[i * 4 + 1] = int(math.copysign(half_w + 0.3, y) * 1000 + SDVIG)
            
    #def byte_put(self, name, i, value):
    #    if value > 65535:
    #        name[i * 2] = 255
    #        name[i * 2 + 1] = 255
    #    else:
    #        name[i * 2] = value % 256
    #        name[i * 2 + 1] = value // 256

    #def byte_get(self, name, i):
    #    return name[i * 2] + name[i * 2 + 1] * 256

    def resampling_wheel(self):
        row = 0
        for i in range(self.n): self.new_p[i] = 0
        index = int(random() * self.n)
        beta = 0.0
        mw = 0
        for i in range(self.n):
            m = self.weights[i]
            if m > mw : mw = m
        for i in range(self.n):
            beta += random() * 2.0 * mw
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index + 1) % self.n
            if self.new_p[index] == 0:  self.new_p[index] = 1
            else: 
                self.new_p[index] += 1
        for el in range(self.n):
            if self.new_p[el] == 0: continue
            #self.tmp[row*8:row*8+6] = self.p[el*8:el*8+6]
            self.tmp[row*4:row*4+3] = self.p[el*4:el*4+3]
            #self.byte_put(self.tmp, row *4, self.p[el, 0])
            #self.byte_put(self.tmp, row *4 + 1, self.p[el, 1])
            #self.byte_put(self.tmp, row *4 + 2, self.p[el, 2])
            #self.byte_put(self.tmp, row *4 + 3, self.byte_get(self.weights, el) * self.byte_get(self.new_p,el))
            self.tmp[row *4 + 3] = self.weights[el] * self.new_p[el]
            #self.tmp[row, 0] = self.p[el, 0]
            #self.tmp[row, 1] = self.p[el, 1]
            #self.tmp[row, 2] = self.p[el, 2]
            #self.tmp[row, 3] = self.byte_get(self.weights, el) * self.byte_get(self.new_p,el)
            row += 1
        return row

    def resampling(self, observations, other_coord):
        #actions = {str(self.count):observations}
        #with open(current_work_directory + "Soccer/Localisation/PF/logs/observations.json", "a", encoding="utf-8") as self.obs_logs:
        #    json.dump(actions, self.obs_logs)
        #actions = {str(self.count):other_coord}
        #with open(current_work_directory + "Soccer/Localisation/PF/logs/other_coord.json", "a", encoding="utf-8") as self.other_logs:
        #    json.dump(actions, self.other_logs)
        for i in range(self.n): 
            #self.byte_put(self.tmp, i *4, 0)
            #self.byte_put(self.tmp, i *4 + 1, 0)
            #self.byte_put(self.tmp, i *4 + 2, 0)
            #self.byte_put(self.tmp, i *4 + 3, 0)
            self.tmp[i *4] = 0
            self.tmp[i *4 + 1] = 0
            self.tmp[i *4 + 2] = 0
            self.tmp[i *4 + 3] = 0
            #self.tmp[i,0] = 0
            #self.tmp[i,1] = 0
            #self.tmp[i,2] = 0
            #self.tmp[i,3] = 0
            #self.byte_put(self.weights, i, 0)
            self.weights[i] = 0 
        
        if used_with_OpenMV:
            clock = time.clock()
            clock.tick()
        S = self.weight_calc_wrap(observations)
        #for i in range(self.n):
        #    weight = int(self.new_observation_score(i,
        #        observations, self.landmarks, self.gauss_noise)*self.new_calc_lines_score(i,
        #        observations['lines'], self.landmarks['lines'], self.line_gauss_noise) * 20000)
        #    self.byte_put(self.weights, i, weight)
        #    S += weight
        #print('self.counter1 = ', self.counter1)
        if used_with_OpenMV: print('timestamp 1 =', clock.avg())
        if S == 0: return
        S = S / 20000
        for i in range(self.n):
            #w = int(self.byte_get(self.weights, i)/S)
            #self.byte_put(self.weights, i, w) 
            w = int(self.weights[i]/S)
            self.weights[i] = w
        row = self.resampling_wheel()
        S = 0.0
        for i in range(row):
            S += (self.tmp[i *4 + 3] / 20000)
        for i in range(row):
            tempo = int(self.tmp[i * 4 + 3]/S)
            self.tmp[i * 4 + 3] = tempo
        #if used_with_OpenMV:
        #    for i in range(row):
        #        S += (tuple(self.tmp[i, 3])[0] / 20000)
        #    for i in range(row):
        #        self.tmp[i, 3] = int(tuple(self.tmp[i, 3])[0] / S )
        #else:
        #    for i in range(row):
        #        S += (self.tmp[i, 3] / 20000)
        #    for i in range(row):
        #        self.tmp[i, 3] = int((self.tmp[i, 3])/ S )
        if not other_coord:
            self.gen_n_particles_robot(row)
        else:
            self.gen_n_particles_robot_coord(row, other_coord)
        #for i in range(row): self.p[i] = self.tmp[i]
        self.p[:row *4] = self.tmp[:row *4]
        #for i in range(row): 
        #    for j in range(4):
        #        self.p[i, j] = self.byte_get(self.tmp, i * 4 + j)
        self.limit_paricles_coord(self.p, self.n)
        self.update_coord(self.p, row)
        self.count += 1
        self.update_consistency(observations)
        #self.logger.step("res",self.return_coord(), self.p)

    def weight_calc_wrap(self, observations):
        if used_with_OpenMV and used_with_OpenMV_firmware:
            array_observation_lines = array.array('f',[])
            for line in observations['lines']:
                array_observation_lines.extend(array.array('f',line))
            array_observations =[]
            for i in range(len(self.land_keys)):
                obs = array.array('f',[])
                for j in range(len(observations[self.land_keys[i]])):
                    ob1 = array.array('f',observations[self.land_keys[i]][j])
                    obs.extend(ob1)
                array_observations.append(obs)
            S = starkit.weight_calculation(self.n, array_observations, self.array_landmarks, self.gauss_noise,
                                           self.p, array_observation_lines, self.landmark_lines_x, self.landmark_lines_y,
                                           self.line_gauss_noise, self.weights)
        else:
            landmarks = self.landmarks
            n = self.n
            p = self.p
            weights = self.weights
            gauss_noise = self.gauss_noise
            line_gauss_noise = self.line_gauss_noise
            weights, S = weight_calculation(n, weights, observations, landmarks, gauss_noise, line_gauss_noise, p)
            self.weights = weights
        #addon_dict = {'weights+': list(weights), 'S': S}
        #data_dict.update(addon_dict)
        #with open("C:/Users/a/source/repos/weight_calc_data.json", "w") as f:
        #        json.dump(data_dict, f)
        return S

    #def weight_calculation(self, n, weights, observations, landmarks, gauss_noise, line_gauss_noise, p):
    #    def byte_put( name, i, value):
    #        name[i * 2] = value % 256
    #        name[i * 2 + 1] = value // 256

    #    def byte_get( name, i):
    #        return name[i * 2] + name[i * 2 + 1] * 256

    #    def gaussian( x, sigma):
    #        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    #        return math.exp(-(x ** 2) / 2*(sigma ** 2)) / math.sqrt(2.0 * math.pi * (sigma ** 2))

    #    def new_observation_score( i, observations, landmarks, gauss_noise, p):
    #        # particle weight calculation
    #        prob = 1.0
    #        part0 = (byte_get(p, i * 4) - SDVIG)/ 1000
    #        part1 = (byte_get(p, i * 4 + 1) - SDVIG)/ 1000
    #        part2 = (byte_get(p, i * 4 + 2) - SDVIG)/ 1000
    #        sin_p = math.sin(part2) 
    #        cos_p = math.cos(part2)
    #        for color_landmarks in observations:
    #            if (color_landmarks not in landmarks):
    #                continue
    #            if (color_landmarks == 'lines'):
    #                continue
    #            for landmark in landmarks[color_landmarks]:
    #                min_dist = 1000
    #                if observations[color_landmarks]:
    #                    for observation in observations[color_landmarks]:
    #                        # calc posts coords in field for every mesurement
    #                        x_posts = part0 + observation[0] * cos_p - observation[1] * sin_p
    #                        y_posts = part1 + observation[0] * sin_p + observation[1] * cos_p
    #                        dist =(x_posts - landmark[0])**2 + (y_posts - landmark[1])**2
    #                        if min_dist > dist:
    #                            min_dist = dist
    #                            weight = observation[2]
    #                if min_dist != 1000:
    #                    prob *= gaussian(math.sqrt(min_dist), gauss_noise)*weight
    #        return prob
    #    def new_calc_lines_score( i, lines, landmarks, line_gauss_noise, p):
    #        '''
    #        line = (ro, theta)
    #        '''
    #        part0 = (byte_get(p, i * 4) - SDVIG)/ 1000
    #        part1 = (byte_get(p, i * 4 + 1) - SDVIG)/ 1000
    #        part2 = (byte_get(p, i * 4 + 2) - SDVIG)/ 1000
    #        prob = 1
    #        if lines != []:
    #            for line in lines:
    #                min_dist = 1000
    #                for landmark_line in landmarks:
    #                    for coord in landmarks[landmark_line]:
    #                        yaw = (part2 + line[1])%(2*math.pi)
    #                        if landmark_line == 'x':
    #                            dist = math.fabs(coord - (part0 + line[0]*math.cos(yaw)))
    #                        else:
    #                            dist = math.fabs(coord - (part1 + line[0]*math.sin(yaw)))
    #                        if min_dist > dist:
    #                            min_dist = dist
    #                if min_dist != 1000:
    #                    prob *= gaussian(min_dist, line_gauss_noise)*line[2]
    #        return prob
    #    S = 0.0
    #    for i in range(n):
    #        weight = int(new_observation_score(i,observations, landmarks, gauss_noise, p)
    #                         *new_calc_lines_score(i,observations['lines'], landmarks['lines'], line_gauss_noise, p)
    #                         * 20000)
    #        byte_put(weights, i, weight)
    #        S += weight
    #    return weights, S



    def new_observation_score(self, i, observations, landmarks, gauss_noise):
        # particle weight calculation
        prob = 1.0
        #if used_with_OpenMV:
        #    part0 = (tuple(self.p[i, 0])[0] - SDVIG)/ 1000
        #    part1 = (tuple(self.p[i, 1])[0] - SDVIG)/ 1000
        #    part2 = (tuple(self.p[i, 2])[0] - SDVIG)/ 1000
        #else: 
        #    part0 = (self.p[i, 0] - SDVIG)/ 1000
        #    part1 = (self.p[i, 1] - SDVIG)/ 1000
        #    part2 = (self.p[i, 2] - SDVIG)/ 1000
        #part0 = (self.byte_get(self.p, i * 4) - SDVIG)/ 1000
        #part1 = (self.byte_get(self.p, i * 4 + 1) - SDVIG)/ 1000
        #part2 = (self.byte_get(self.p, i * 4 + 2) - SDVIG)/ 1000
        part0 = (self.p[i * 4] - SDVIG)/ 1000
        part1 = (self.p[i * 4 + 1] - SDVIG)/ 1000
        part2 = (self.p[i * 4 + 2] - SDVIG)/ 1000
        sin_p = math.sin(part2) 
        cos_p = math.cos(part2)
        for color_landmarks in observations:
            if (color_landmarks not in landmarks):
                continue
            if (color_landmarks == 'lines'):
                continue
            for landmark in landmarks[color_landmarks]:
                #dists_number = 0
                min_dist = 1000
                if observations[color_landmarks]:
                    for observation in observations[color_landmarks]:
                        # calc posts coords in field for every mesurement
                        x_posts = part0 + observation[0] * cos_p - observation[1] * sin_p
                        y_posts = part1 + observation[0] * sin_p + observation[1] * cos_p
                        dist =(x_posts - landmark[0])**2 + (y_posts - landmark[1])**2
                        if min_dist > dist:
                            min_dist = dist
                            weight = observation[2]
                        #self.new_p[dists_number] = int(dist * 1000)
                        #dists_number += 1
                        self.counter1 += 1
                #if dists_number != 0:
                #    prob *= gaussian(np.min(self.new_p[:dists_number]) / 1000, gauss_noise)\
                #            *observations[color_landmarks][np.argmin(self.new_p[:dists_number])][2]
                if min_dist != 1000:
                    prob *= gaussian(math.sqrt(min_dist), gauss_noise)*weight
        return prob



    def new_calc_lines_score(self, i, lines, landmarks, gauss_noise):
        '''
        line = (ro, theta)
        '''
        #if used_with_OpenMV:
        #    part0 = (tuple(self.p[i, 0])[0] - SDVIG)/ 1000
        #    part1 = (tuple(self.p[i, 1])[0] - SDVIG)/ 1000
        #    part2 = (tuple(self.p[i, 2])[0] - SDVIG)/ 1000
        #else: 
        #    part0 = (self.p[i, 0] - SDVIG)/ 1000
        #    part1 = (self.p[i, 1] - SDVIG)/ 1000
        #    part2 = (self.p[i, 2] - SDVIG)/ 1000
        #part0 = (self.byte_get(self.p, i * 4) - SDVIG)/ 1000
        #part1 = (self.byte_get(self.p, i * 4 + 1) - SDVIG)/ 1000
        #part2 = (self.byte_get(self.p, i * 4 + 2) - SDVIG)/ 1000
        part0 = (self.p[i * 4] - SDVIG)/ 1000
        part1 = (self.p[i * 4 + 1] - SDVIG)/ 1000
        part2 = (self.p[i * 4 + 2] - SDVIG)/ 1000
        prob = 1
        if lines != []:
            for line in lines:
                #dists_number = 0
                min_dist = 1000
                for landmark_line in landmarks:
                    for coord in landmarks[landmark_line]:
                        yaw = (part2 + line[1])%(2*math.pi)
                        if landmark_line == 'x':
                            dist = math.fabs(coord - (part0 + line[0]*math.cos(yaw)))
                        else:
                            dist = math.fabs(coord - (part1 + line[0]*math.sin(yaw)))
                        #self.new_p[dists_number] = int(dist * 1000)
                        #dists_number += 1
                        if min_dist > dist:
                            min_dist = dist
                #if dists_number != 0:
                #    prob *= gaussian(np.min(self.new_p[:dists_number]) / 1000, gauss_noise)*line[2]
                if min_dist != 1000:
                    prob *= gaussian(min_dist, gauss_noise)*line[2]
        return prob

    def custom_reset(self, x, y, yaw):
        self.myrobot.x = x
        self.myrobot.y = y
        self.myrobot.yaw = yaw
        self.p = gen_n_particles_robot(0)

    # ------------------------------
    # need to add to handle the fall
    # ------------------------------

    def fall_reset(self, noise = 0.3):
        self.myrobot.x += gauss(0, self.sense_noise)
        self.myrobot.y += gauss(0, self.sense_noise)
        self.myrobot.yaw += gauss(0, self.yaw_noise)
        for i in range(self.n):
            #if used_with_OpenMV:
            #    self.p[i, 0] = tuple(self.p[i, 0])[0] + int(gauss(0, noise) * 1000)
            #    self.p[i, 1] = tuple(self.p[i, 1])[0] + int(gauss(0, noise) * 1000)
            #    self.p[i, 2] = tuple(self.p[i, 2])[0] + int(gauss(0, noise / 3) * 1000)
            #else:
            #    self.p[i, 0] = self.p[i, 0] + int(gauss(0, noise) * 1000)
            #    self.p[i, 1] = self.p[i, 1] + int(gauss(0, noise) * 1000)
            #    self.p[i, 2] = self.p[i, 2] + int(gauss(0, noise / 3) * 1000)
            #x1 = self.byte_get(self.p, i * 4) + int(gauss(0, noise) * 1000)
            #y1 = self.byte_get(self.p, i * 4 + 1) + int(gauss(0, noise) * 1000)
            #yaw1 = self.byte_get(self.p, i * 4 + 2) + int(gauss(0, noise / 3) * 1000)
            #self.byte_put(self.p, i * 4, x1)
            #self.byte_put(self.p, i * 4 +1, y1)
            #self.byte_put(self.p, i * 4 + 2, yaw1)
            #self.p[i, 0] = x1
            #self.p[i, 1] = y1
            #self.p[i, 2] = yaw1
            x1 = self.p[i * 4] + int(gauss(0, noise) * 1000)
            y1 = self.p[i * 4 + 1] + int(gauss(0, noise) * 1000)
            yaw1 = self.p[i * 4 + 2] + int(gauss(0, noise / 3) * 1000)
            self.p[i * 4] = x1
            self.p[i * 4 +1] = y1
            self.p[i * 4 + 2] = yaw1
        self.consistency *= 0.5

    def update_coord(self, particles, stop_row):
        x = 0.0
        y = 0.0
        orientation = 0.0
        adding = 0.0
        if (self.myrobot.yaw < math.pi/2) or (self.myrobot.yaw > math.pi*3/2):
            adding = math.pi*2
        for i in range(stop_row):
            #if used_with_OpenMV:
            #    x += (tuple(particles[i, 0])[0] - SDVIG) / 1000 * tuple(particles[i, 3])[0] / 20000
            #    y += (tuple(particles[i, 1])[0] - SDVIG) / 1000 * tuple(particles[i, 3])[0] / 20000
            #    if ((tuple(particles[i, 2])[0] - SDVIG) / 1000 < math.pi):
            #        culc_yaw = (tuple(particles[i, 2])[0] - SDVIG) / 1000 + adding
            #    else:
            #        culc_yaw = (tuple(particles[i, 2])[0] - SDVIG) / 1000
            #    orientation += culc_yaw * tuple(particles[i, 3])[0] / 20000
            #else:
            #    x += (particles[i, 0] - SDVIG) / 1000 * particles[i, 3] / 20000
            #    y += (particles[i, 1] - SDVIG) / 1000 * particles[i, 3] / 20000
            #    if ((particles[i, 2] - SDVIG) / 1000 < math.pi):
            #        culc_yaw = (particles[i, 2] - SDVIG) / 1000 + adding
            #    else:
            #        culc_yaw = (particles[i, 2] - SDVIG) / 1000
            #    orientation += culc_yaw * particles[i, 3] / 20000
            #x_old = (self.byte_get(particles, i * 4)- SDVIG) / 1000
            #y_old = (self.byte_get(particles, i * 4 + 1)- SDVIG) / 1000
            #yaw_old = (self.byte_get(particles, i * 4 + 2)- SDVIG) / 1000
            #w_old = self.byte_get(particles, i * 4 + 3) / 20000
            x_old = (particles[i * 4]- SDVIG) / 1000
            y_old = (particles[i * 4 + 1]- SDVIG) / 1000
            yaw_old = (particles[i * 4 + 2]- SDVIG) / 1000
            w_old = particles[i * 4 + 3] / 20000
            x += x_old * w_old
            y += y_old * w_old
            if (yaw_old < math.pi):
                culc_yaw = yaw_old + adding
            else:
                culc_yaw = yaw_old
            orientation += culc_yaw * w_old
        self.myrobot.x = x
        self.myrobot.y = y
        self.myrobot.yaw = orientation % (2*math.pi)

    def return_coord(self):
        return self.myrobot.x, self.myrobot.y, self.myrobot.yaw

    def updatePF(self, measurement, other_coord):
        for i in range(self.number_of_res):
            self.resampling(measurement, other_coord)
        return self.return_coord()

class Robot:

    def __init__(self, x=0, y=0, yaw=0):
        self.x = x          # robot's x coordinate
        self.y = y          # robot's y coordinate
        self.yaw = yaw      # robot's angle

    def move(self, x, y, yaw):
        # turn, and add randomomness to the turning command
        orientation = self.yaw + float(yaw)
        if orientation < 0:
            orientation += (math.pi*2)
        orientation %= (2 * math.pi)
        self.x += x*math.cos(self.yaw) - y*math.sin(self.yaw)
        self.y += x*math.sin(self.yaw) + y*math.cos(self.yaw)
        self.yaw = orientation






