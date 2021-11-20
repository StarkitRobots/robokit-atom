

import sys, os, cv2, copy
import numpy as np

sys.path.append("/home/kondo/kondo_atom")
from show_mask import show_mask_and_annotated_objects


#current_work_directory = os.getcwd()
#current_work_directory = current_work_directory.replace('\\', '/')
#if sys.version != '3.4.0':
    #current_work_directory += '/'
    #import numpy as np
#else:
    #import ulab as np
    #import sensor, time

#sys.path.append( current_work_directory + 'Soccer/')
#sys.path.append( current_work_directory + 'Soccer/Motion/')
#sys.path.append( current_work_directory + 'Soccer/Vision/')
#sys.path.append( current_work_directory + 'Soccer/Localisation/')
#sys.path.append( current_work_directory + 'Soccer/Localisation/PF/')

#import class_Motion
import math, time, json, array
from call_par_filter import Call_Par_Filter
from ParticleFilter import random

LOCALISATION_VISUALISATION_IS_ON = True
OBSTACLE_VISUALISATION_IS_ON = False
LOG_PF_DEVIATION_IS_ON = False



def uprint(*text):
    #with open(current_work_directory + "Soccer/log/output.txt",'a') as f:
    #    print(*text, file = f)
    print(*text )

class M_blob:
    def __init__ (self, x_, y_, w_, h_):
        self.x_ = x_
        self.y_ = y_
        self.w_ = w_
        self.h_ = h_

    def x (self):
        return self.x_

    def y (self):
        return self.y_

    def w (self):
        return self.w_

    def h (self):
        return self.h_

    def cx (self):
        return int (self.x_ + self.w_ / 2)

    def cy (self):
        return int (self.y_ + self.h_ / 2)

    def rect (self):
        return (self.x_, self.y_, self.w_, self.h_)

class Local():
    def __init__ (self, motion,glob, vision, coord_odometry = [0.0,0.0,0.0]):
        self.motion = motion
        self.glob = glob
        self.USE_LANDMARKS_FOR_LOCALISATION = self.glob.params['USE_LANDMARKS_FOR_LOCALISATION']
        self.USE_LINES_FOR_LOCALISATION = self.glob.params["USE_LINES_FOR_LOCALISATION"]
        self.USE_LINE_CROSSES_FOR_LOCALISATION = self.glob.params["USE_LINE_CROSSES_FOR_LOCALISATION"]
        self.USE_PENALTY_MARKS_FOR_LOCALISATION = self.glob.params["USE_PENALTY_MARKS_FOR_LOCALISATION"]
        self.DIRECT_COORD_MEASUREMENT_BY_PAIRS_OF_POST = self.glob.params["DIRECT_COORD_MEASUREMENT_BY_PAIRS_OF_POST"]
        self.USE_SINGLE_POST_MEASUREMENT = self.glob.params["USE_SINGLE_POST_MEASUREMENT"]
        self.floor_lines = []                                   # detected raw lines on floor [rho:meter, theta: radians] in local robot coordinates
        self.line_group_compact = []                            # grouped lines [rho:meter, theta: radians] in local robot coordinates
        self.cross_points = []                                  # detected cross points of lines on floor [meter,meter] in local robot coordinates
        #self.post_data_in_pose = []
        self.coordinate = [0.0,0.0,0.0]
        self.coord_visible = [0.0,0.0,0.0]
        self.coord_odometry = coord_odometry
        self.coord_odometry_old = coord_odometry
        self.quality = 0
        self.vision = vision
        self.penalty = []
        self.penalty_points =[]
        self.robot_moved = False
        self.coord_shift = [0.0, 0.0, 0.0]
        self.field_border_data = []
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 0 or self.glob.SIMULATION == 3 or self.glob.SIMULATION == 5:
            import reload as re
            import cv2, copy
            import numpy as np
            self.np =np
            self.copy = copy
            self.re = re
            self.cv2 = cv2
            self.timer0 = time.perf_counter()
            if abs(motion.direction_To_Attack) < 1: self.side_factor = 1
            else: self.side_factor = -1

            if self.glob.SIMULATION != 5:
                from class_Visualisation import Visualisation
                self.visualisation = Visualisation()
            self.pf_deviation = [0,0]
            self.ab_deviation = [0,0]
        elif self.glob.SIMULATION == 2:
            import sensor
            import image
            #import micropython, gc
            import pyb as p
            import ulab as np
            self.pyb = p
            self.image = image
            self.sensor = sensor
            self.timer0 = p.millis()
            #gc.collect()
            #micropython.mem_info()
        self.call_Par_Filter = Call_Par_Filter(self.glob, coord_odometry)
        self.max_field_dimension = math.sqrt(self.glob.landmarks['FIELD_WIDTH']**2 + self.glob.landmarks['FIELD_LENGTH']**2)* 1.2
        self.post_data_in_pose = np.zeros((160,3), dtype = np.int16)  # (course (rad) * 2000, distance (m) * 2000, post Number)
        self.post_data_in_pose_number = 0

    def detect_Post_In_image(self,img_, post_color):            # color: 1 - blue, 2- yellow
        all_goal_is_in_picture = 0
        post_list = []
        while (all_goal_is_in_picture < 2):
            if self.glob.SIMULATION == 2 :
                img = img_
            else:
                img = self.re.Image(img_)
                #print(type(img_))
                #try:
                #    img = 
                #except:
                #    img = self.re.Image(copy.deepcopy(img_.img))
                #img.rotate()
                #img = self.re.Image(copy.deepcopy(img_.img))
                #img = img_
                #cv2.imshow('01', img_)
                labimg = self.cv2.cvtColor (img_, self.cv2.COLOR_BGR2LAB)
                
                if self.glob.SIMULATION == 0 or self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                    self.motion.vision_Sensor_Display( img_)
            if all_goal_is_in_picture == 1 :
                all_goal_is_in_picture = all_goal_is_in_picture + 1
            #if self.glob.SIMULATION == 2 :
            post_thresholds =  [self.vision.TH[post_color]['th']]
            #else: post_thresholds =  self.vision.TH[post_color]['th']
            for blob in img.find_blobs(post_thresholds, pixels_threshold = self.vision.TH[post_color]['pixel'],
                                      area_threshold = self.vision.TH[post_color]['area'], merge=True):
                blob_Is_Post = False
                if blob.y() + blob.h() > 235 : continue         # blob connected to bottom of picture. No opportunity to recognize data
                else:
                    if blob.w() > 314: continue         # blob connected to both sides of picture. No opportunity to recognize data
                    for y in range (blob.y() + blob.h(), blob.y() + blob.h() + 5, 1 ):
                        for x in range (blob.x(), blob.x() + blob.w(), 1 ):
                            if self.glob.SIMULATION == 2 :
                                a=self.image.rgb_to_lab(img.get_pixel(x , y))
                                is_green = (self.vision.TH['green field']['th'][0] < a[0] < self.vision.TH['green field']['th'][1]) and \
                                           (self.vision.TH['green field']['th'][2] < a[1] < self.vision.TH['green field']['th'][3]) and \
                                           (self.vision.TH['green field']['th'][4] < a[2] < self.vision.TH['green field']['th'][5])
                                is_white = (self.vision.TH['white marking']['th'][0] < a[0] < self.vision.TH['white marking']['th'][1]) and \
                                           (self.vision.TH['white marking']['th'][2] < a[1] < self.vision.TH['white marking']['th'][3]) and \
                                           (self.vision.TH['white marking']['th'][4] < a[2] < self.vision.TH['white marking']['th'][5])
                            else:
                                is_green = (self.vision.TH['green field']['th'][0] < labimg[y][x][0]/ 2.55 <  self.vision.TH['green field']['th'][1]) and \
                                           (self.vision.TH['green field']['th'][2] < labimg[y][x][1]- 128 <  self.vision.TH['green field']['th'][3]) and \
                                           (self.vision.TH['green field']['th'][4] < labimg[y][x][2]- 128 <  self.vision.TH['green field']['th'][5])
                                is_white = (self.vision.TH['white marking']['th'][0] < labimg[y][x][0]/ 2.55 <  self.vision.TH['white marking']['th'][1]) and \
                                           (self.vision.TH['white marking']['th'][2] < labimg[y][x][1]- 128 <  self.vision.TH['white marking']['th'][3]) and \
                                           (self.vision.TH['white marking']['th'][4] < labimg[y][x][2]- 128 <  self.vision.TH['white marking']['th'][5])
                            if is_green == True or is_white == True : blob_Is_Post = True
                            if blob_Is_Post == True: break
                        if blob_Is_Post == True: break
                    if blob_Is_Post == True:
                        blob_y_plus_h = blob.y() + blob.h()
                        y = blob_y_plus_h - 2
                        post_color_pixels = []
                        for x in range (blob.x(), blob.x() + blob.w(), 1 ):
                            if self.glob.SIMULATION == 2 :
                                is_post_color = self.vision.TH[post_color]['th'][0] < a[0] < self.vision.TH[post_color]['th'][1] and self.vision.TH[post_color]['th'][2] < a[1] < self.vision.TH[post_color]['th'][3] and self.vision.TH[post_color]['th'][4] < a[2] < self.vision.TH[post_color]['th'][5]
                            else:
                                is_post_color = self.vision.TH[post_color]['th'][0] < labimg[y][x][0]/ 2.55 <  self.vision.TH[post_color]['th'][1] and self.vision.TH[post_color]['th'][2] < labimg[y][x][1]- 128 <  self.vision.TH[post_color]['th'][3] and self.vision.TH[post_color]['th'][4] < labimg[y][x][2]- 128 <  self.vision.TH[post_color]['th'][5]
                            if is_post_color == True: post_color_pixels.append(x)
                        if len(post_color_pixels) == 0: blob_cx = blob.cx()
                        else: blob_cx = int((post_color_pixels[0] + post_color_pixels[len(post_color_pixels)-1])/2)
                        course, dist = self.motion.get_course_and_distance_to_post( blob_cx, blob_y_plus_h )
                        if dist > self.max_field_dimension : continue                           # filter off too far measurements
                        distance_in_mm = dist *1000
                        virtual_width_of_post = distance_in_mm* math.tan(math.radians(blob.w() * self.motion.params['APERTURE_PER_PIXEL']))
                        if virtual_width_of_post < self.motion.params['POST_WIDTH_IN_MM'] * 1.5 :
                            course = self.normalize_yaw( course )
                            post_list.append([course, dist, 1]) # recognized post
                        else:
                            if blob.x() < 3 or all_goal_is_in_picture == 2:
                                course = self.normalize_yaw( course - math.radians(blob.w() * self.motion.params['APERTURE_PER_PIXEL']/2))
                                post_list.append([course , dist, 2])  # blob connected to left side of picture
                            else:
                                if blob.x() + blob.w() > 317 or all_goal_is_in_picture == 3:
                                    course = self.normalize_yaw( course + math.radians(blob.w() * self.motion.params['APERTURE_PER_PIXEL']/2))
                                    post_list.append([course , dist, 3]) # blob connected to right side of picture
                                else:
                                    if all_goal_is_in_picture == 0:
                                        all_goal_is_in_picture =1
                                        if self.glob.SIMULATION == 2 :
                                            for y in range(0,blob.cy(),1):
                                                for x in range(320):
                                                    img_.set_pixel(x,y,(0,0,0))
                                        else:
                                            for y in range(0,blob.cy(),1):
                                                for x in range(320):
                                                    img_[y][x] =  img_[y][x]*0

            if all_goal_is_in_picture == 0: break
        #uprint('posts number in frame = ', len(post_list))
        if len(post_list) > 2:
            post_list = self.group_posts_by_yaw(post_list)
        return post_list

    def group_posts_by_yaw(self, post_list):
        posts = []
        for ind in range(2):
            best_post_index = 0
            best_score = 0
            len_p = len(post_list)
            for i in range(len_p):
                score = 0
                for j in range(len_p):
                    if abs(post_list[i][0] - post_list[j][0]) < 0.1: score += 1
                if score > best_score:
                    best_score = score
                    best_post_index = i
            if len_p != 0:
                posts.append(post_list[best_post_index])
                k = 0
                reference = post_list[best_post_index][0]
                for i in range(len_p):
                    if abs( reference - post_list[i - k][0] ) < 0.1:
                        post_list.pop(i - k)
                        k += 1
        return posts





    def normalize_yaw(self, yaw):
        if abs(yaw) > 2 * math.pi: yaw %= (2 * math.pi)
        if yaw > math.pi : yaw -= (2 * math.pi)
        if yaw < -math.pi : yaw += (2 * math.pi)
        return yaw


    def read_Localization_marks(self, img):
        #cv2.imshow('03', img)


        self.quality = 0
        post_list1 = self.detect_Post_In_image(img, "blue posts")
        for post in post_list1:
            self.post_data_in_pose[self.post_data_in_pose_number][0] = int(post[0] * 2000)
            self.post_data_in_pose[self.post_data_in_pose_number][1] = int(post[1] * 2000)
            self.post_data_in_pose_number += 1
        #if len(post_list1) != 0:
        #    self.post_data_in_pose.extend(post_list1)
        post_list2 = self.detect_Post_In_image(img, "yellow posts")
        for post in post_list2:
            self.post_data_in_pose[self.post_data_in_pose_number][0] = int(post[0] * 2000)
            self.post_data_in_pose[self.post_data_in_pose_number][1] = int(post[1] * 2000)
            self.post_data_in_pose_number += 1
        #if len(post_list2) != 0:
        #    self.post_data_in_pose.extend(post_list2)
        if self.USE_LANDMARKS_FOR_LOCALISATION == True:
            if self.USE_PENALTY_MARKS_FOR_LOCALISATION ==True: self.detect_penalty_marks(img)
            if self.USE_LINES_FOR_LOCALISATION == True : self.detect_line_in_image(img)
        if self.robot_moved == True:
            self.robot_moved = False
            self.glob.obstacles.clear()
        if self.glob.obstacleAvoidanceIsOn:
            self.detect_obstacles(img)


    def pf_update(self, coord_visible_is_new):
        if self.USE_LANDMARKS_FOR_LOCALISATION == True:
            if self.USE_LINES_FOR_LOCALISATION == True : self.group_lines_find_crosses()
            if self.USE_PENALTY_MARKS_FOR_LOCALISATION ==True: self.group_penalty_marks()
        landmarks ={}
        correction = -self.motion.imu_body_yaw()
        post1_data = []
        post2_data = []
        post3_data = []
        post4_data = []
        unsorted_data = []
        if self.USE_SINGLE_POST_MEASUREMENT == True:
            #for post in self.post_data_in_pose :
            for i in range(self.post_data_in_pose_number):
                post = tuple(self.post_data_in_pose[i])
                x = post[1]*math.cos(post[0] / 2000 + correction) / 2000
                y = post[1]*math.sin(post[0] / 2000 + correction) / 2000
                weight = 1
                #if x*x+y*y > 6.25: weight = 0.5
                if post[2] == 0 : unsorted_data.append([x,y,weight])
                if post[2] == 1 : post1_data.append([x,y,weight])
                if post[2] == 2:  post2_data.append([x,y,weight])
                if post[2] == 3:  post3_data.append([x,y,weight])
                if post[2] == 4:  post4_data.append([x,y,weight])
        landmarks['unsorted_posts']= unsorted_data
        landmarks['post1']= post1_data
        landmarks['post2']= post2_data
        landmarks['post3']= post3_data
        landmarks['post4']= post4_data
        landmarks['lines'] = []
        landmarks['Line_crosses'] = []
        landmarks['penalty'] = []
        if self.USE_LANDMARKS_FOR_LOCALISATION == True:
            if self.USE_LINES_FOR_LOCALISATION ==True:landmarks['lines'] = self.line_group_compact
            if self.USE_LINE_CROSSES_FOR_LOCALISATION ==True:landmarks['Line_crosses'] = self.cross_points
            if self.USE_PENALTY_MARKS_FOR_LOCALISATION ==True: landmarks['penalty'] = self.penalty_points
            #uprint('self.cross_points = ', self.cross_points)
            #for i in range(len(self.line_group_compact)): print('rho = ', self.line_group_compact[i][0], '\t', 'theta =', math.degrees(self.line_group_compact[i][1]))
        uprint('Number of cross_points = ', len(self.cross_points), 'lines =', len(self.line_group_compact),
               'penalty =', len(self.penalty_points), 'posts =',
               len(unsorted_data)+len(post1_data)+len(post2_data)+len(post3_data)+len(post4_data))
        coord = []
        if coord_visible_is_new == True:
            coord.append(self.coord_visible[0])
            coord.append(self.coord_visible[1])
            coord.append(self.coord_visible[2])
        if self.DIRECT_COORD_MEASUREMENT_BY_PAIRS_OF_POST == False: coord.clear()                   #!!!!!!!!
        self.call_Par_Filter.update(landmarks, coord)
        if self.glob.SIMULATION == 2 or self.glob.SIMULATION == 5:
            uprint( 'coord =', coord)
        else:
            if self.motion.robot_Number == '':
                uprint( 'coord =', coord)
        #with open(current_work_directory + "Soccer/log/pf_data.json",'a') as f:
        #    print(landmarks, ',{"coord":', coord, '},', file = f)


    def correct_yaw_in_pf(self):
        self.glob.pf_coord = self.call_Par_Filter.return_coord()
        self.motion.refresh_Orientation()
        shift__Yaw = self.motion.imu_body_yaw() -  self.glob.pf_coord[2]
        shifts = {'shift_x': 0, 'shift_y': 0,'shift_yaw': shift__Yaw}
        self.call_Par_Filter.move(shifts)
        self.coord_odometry[2] = self.motion.imu_body_yaw()
        self.glob.pf_coord = self.call_Par_Filter.return_coord()



    def coordinate_record(self, odometry = False, shift = False):
        self.glob.pf_coord = self.call_Par_Filter.return_coord()
        if odometry == True:
            if shift == False:
                shift__X = (self.coord_odometry[0] -self.coord_odometry_old[0])
                shift__Y = (self.coord_odometry[1] -self.coord_odometry_old[1])
                shift__Yaw = 0
                shift__X_local = shift__X * math.cos(self.glob.pf_coord[2]) + shift__Y * math.sin(self.glob.pf_coord[2])
                shift__Y_local = - shift__X * math.sin(self.glob.pf_coord[2]) + shift__Y * math.cos(self.glob.pf_coord[2])
                self.coord_odometry_old = self.coord_odometry.copy()
                self.coordinate = self.coord_odometry.copy()
            else:
                shift__X_local = self.coord_shift[0]
                shift__Y_local = self.coord_shift[1]
                shift__Yaw = self.coord_shift[2]
            shifts = {'shift_x': shift__X_local, 'shift_y': shift__Y_local, 'shift_yaw': shift__Yaw}
            self.call_Par_Filter.move(shifts)
            self.robot_moved = True
            self.cross_points.clear()
            self.penalty_points.clear()
        self.correct_yaw_in_pf()
        self.glob.pf_coord = self.call_Par_Filter.return_coord()
        if (self.glob.SIMULATION == 1 or self.glob.SIMULATION == 0 or self.glob.SIMULATION == 3):
            timer1 = time.perf_counter() - self.timer0
            Dummy_PF_position = [self.glob.pf_coord[0] * self.side_factor,
                                self.glob.pf_coord[1] * self.side_factor, 0.01]
            Dummy_PF_orientation = [0, 0, self.glob.pf_coord[2] + (math.pi/2 *(1 - self.side_factor))]
            returnCode, Dummy_1position= self.motion.sim.simxGetObjectPosition(
                                         self.motion.clientID, self.motion.Dummy_1Handle ,
                                         -1, self.motion.sim.simx_opmode_streaming)
            Dummy_PF_H = []
            yaw_rad = math.radians(self.motion.body_euler_angle[0])
            if self.USE_LANDMARKS_FOR_LOCALISATION == True:
                n = len(self.cross_points)
                m = len(self.penalty_points)
                for i in range(n+m):
                    Dummy_PF_H_position = []
                    if i < n:
                        Dummy_PF_H_position.append((Dummy_1position[0] + self.cross_points[i][0]*math.cos(yaw_rad) -
                                                 self.cross_points[i][1]*math.sin(yaw_rad)))
                        Dummy_PF_H_position.append((Dummy_1position[1] + self.cross_points[i][0]*math.sin(yaw_rad) +
                                                 self.cross_points[i][1]*math.cos(yaw_rad)))
                    elif i < n+m and self.USE_PENALTY_MARKS_FOR_LOCALISATION ==True:
                        Dummy_PF_H_position.append((Dummy_1position[0] + self.penalty_points[i-n][0]*math.cos(yaw_rad) -
                                                 self.penalty_points[i-n][1]*math.sin(yaw_rad)))
                        Dummy_PF_H_position.append((Dummy_1position[1] + self.penalty_points[i-n][0]*math.sin(yaw_rad) +
                                                 self.penalty_points[i-n][1]*math.cos(yaw_rad)))
                    Dummy_PF_H_position.append(0)
                    Dummy_PF_H.append(Dummy_PF_H_position)


            if LOCALISATION_VISUALISATION_IS_ON :
                self.visualisation.localisation_points(self.motion, Dummy_PF_position, Dummy_PF_orientation, Dummy_PF_H)
            if OBSTACLE_VISUALISATION_IS_ON and self.glob.obstacleAvoidanceIsOn and odometry == False:
                scene_obstacles = []
                for i in range(len(self.glob.obstacles)):
                    obstacle_for_scene = []
                    obstacle_for_scene.append((self.glob.obstacles[i][0] + Dummy_1position[0]) * self.side_factor - self.glob.pf_coord[0])
                    obstacle_for_scene.append((self.glob.obstacles[i][1] + Dummy_1position[1]) * self.side_factor - self.glob.pf_coord[1])
                    obstacle_for_scene.append(self.glob.obstacles[i][2])
                    scene_obstacles.append(obstacle_for_scene)
                self.visualisation.obstacle_mark(self.motion, scene_obstacles)
            if odometry == False and LOG_PF_DEVIATION_IS_ON:
                with open(self.glob.current_work_directory + "Soccer/log/deviation.json",'a') as f:
                    self.call_Par_Filter.pf.timer = int(timer1)
                    pf_deviation = math.sqrt((Dummy_1position[0] - Dummy_PF_position[0])**2 + (Dummy_1position[1] - Dummy_PF_position[1])**2)
                    self.pf_deviation[0] += 1
                    self.pf_deviation[1] = (self.pf_deviation[1] *(self.pf_deviation[0]-1) + pf_deviation)/self.pf_deviation[0]
                    print('{','"average pf":', self.pf_deviation[1], ', "current pf":', pf_deviation,'},', file = f)

        elif self.glob.SIMULATION == 2 :
            timer1 = self.pyb.elapsed_millis(self.timer0)/1000
        #with open(current_work_directory + "Soccer/log/pf_data.json",'a') as f:
        #    print('{"time":', round(timer1,0),
        #         ',"glob.pf_coord":',
        #         [round(self.glob.pf_coord[0], 2), round(self.glob.pf_coord[1], 2), round(self.glob.pf_coord[2], 2)],
        #         ',"glob.ball_coord":',
        #         [round(self.glob.ball_coord[0], 2), round(self.glob.ball_coord[1], 2)],
        #         '},', file = f)

    def localisation_Complete(self):
        returncode = self.process_Post_data_in_Pose()
        self.pf_update(returncode)
        alpha, betta = 0.3, 0.7
        if abs(self.coord_visible[0]) > 2 or abs(self.coord_visible[1]) > 1.5 : alpha, betta = 0,1
        self.coordinate[0] = (self.coord_visible[0] * alpha +self.coord_odometry[0] * betta)
        self.coordinate[1] = (self.coord_visible[1] * alpha +self.coord_odometry[1] * betta)
        self.coordinate[2] = (self.coord_visible[2] * alpha +self.coord_odometry[2] * betta)
        deviation = math.sqrt((self.coord_visible[0] -self.coord_odometry[0])**2 +  (self.coord_visible[1] -self.coord_odometry[1])**2)
        if deviation < 0.1: self.quality = 1
        else: self.quality = 0.1/deviation
        #self.coord_odometry = self.coordinate.copy()
        self.post_data_in_pose_number = 0
        self.correct_yaw_in_pf()
        self.glob.pf_coord = self.call_Par_Filter.return_coord()
        if self.glob.obstacleAvoidanceIsOn: self.group_obstacles()
        self.coordinate_record()
        return returncode

    def process_Post_data_in_Pose(self):
        if self.post_data_in_pose_number < 2: return False
        #uprint('len(self.post_data_in_pose) = ', len(self.post_data_in_pose))
        post_data = []
        for i in range (self.post_data_in_pose_number):                                           #|
            a = [list(self.post_data_in_pose[i])[0] / 2000,
                 list(self.post_data_in_pose[i])[1] / 2000,
                 list(self.post_data_in_pose[i])[2]]                                             #|Copy class data into method
            post_data.append(a)                                                                 #|
        sorted_data = sorted(post_data)
        max_difference = 0
        del post_data
        max_difference_index = 0
        for i in range(len(sorted_data)-1):                                                     #|
            if sorted_data[i+1][0]- sorted_data[i][0] > max_difference:                         #|find index of post in sorted list with biggest
                max_difference = sorted_data[i+1][0]- sorted_data[i][0]                         #| difference in yaw
                max_difference_index = i                                                        #|
        divider = sorted_data[max_difference_index][0] + max_difference/2                           # define yaw andgle of divider
        for i in range(len(sorted_data)):                                               #| rotate all data in order to put biggest difference into gap -180/+180
            sorted_data[i][0] = sorted_data[i][0] - divider + math.pi                        #| !!!
            if sorted_data[i][0] > math.pi : sorted_data[i][0] -= ( 2 * math.pi )            #|
        new_data = sorted(sorted_data)                                                      # sort all data again
        del sorted_data
        differences = []                                                                #|
        for i in range(len(new_data)-1):                                                #| create list of differences in yaw with index
            differ = new_data[i+1][0] - new_data[i][0]                                  #|
            if differ > 2 *math.pi: differ %= (2 * math.pi)                             #|
            if differ > math.pi: differ = 2 * math.pi - differ                          #|
            differences.append((differ, i))                                             #|
        differences_descending = sorted(differences,  reverse = True)                   #|
        del differences
        visible_posts_number = 4                                                                                #|
        if len(differences_descending) > 2 and differences_descending[2][0] <= 0.14: visible_posts_number = 3      #|
        if len(differences_descending) > 2 and differences_descending[1][0] <= 0.14: visible_posts_number = 2      #|
        if len(differences_descending) > 2 and differences_descending[0][0] <= 0.14: visible_posts_number = 1      #|
        if len(differences_descending) == 2 :                                                                   #|detect visible posts number
            visible_posts_number = 3                                                                            #|
            if differences_descending[1][0] <= 0.14: visible_posts_number = 2                                      #|
            if differences_descending[0][0] <= 0.14: visible_posts_number = 1                                      #|
        if len(differences_descending) == 1:                                                                    #|
            visible_posts_number = 2                                                                            #|
            if differences_descending[0][0] <= 0.14: visible_posts_number = 1                                      #|
        if visible_posts_number == 1: return False                                                              #|
        windows = []
        for i in range(visible_posts_number - 1): windows.append(differences_descending[i][1]+1)
        del differences_descending
        organized_windows = sorted(windows)
        del windows
        organized_windows.append(len(new_data))
        posts = []
        yaw_p = 0
        dist_p = 0
        index_p = 0
        for i in range( len(organized_windows)):
            yaw = 0
            dist = 0
            for j in range (organized_windows[i]):
                yaw = yaw + new_data[j][0]
                dist = dist + new_data[j][1]
            posts.append([(yaw-yaw_p)/(organized_windows[i] -index_p) + divider - math.pi, (dist-dist_p)/(organized_windows[i]-index_p),0])
            yaw_p = yaw
            dist_p = dist
            index_p = organized_windows[i]
        del new_data
        del organized_windows
        for i in range(len(posts)):
            yaw = posts[i][0]
            yaw = self.normalize_yaw( yaw )
            posts[i][0] = yaw
        if visible_posts_number == 4:
            imin = 0
            for i in range(4):
                if abs(posts[i][0]) < abs(posts[imin][0]): imin = i
            if imin == 0: imin2 = 1
            else: imin2 = 0
            for i in range(4):
                if i == imin : continue
                if abs(posts[i][0]) < abs(posts[imin2][0]): imin2 = i
            if posts[imin][0] > posts[imin2][0]:
                posts[imin2][2] = 1
                posts[imin][2] = 2
                guest_right_post_index = imin2
            else:
                posts[imin][2] = 1
                posts[imin2][2] = 2
                guest_right_post_index = imin
            home_right_post_index = guest_right_post_index + 2
            home_left_post_index = guest_right_post_index + 3
            if home_right_post_index > 0: home_right_post_index = home_right_post_index - 4
            if home_left_post_index > 0: home_left_post_index = home_left_post_index - 4
            posts[home_right_post_index][2] = 3
            posts[home_left_post_index][2] = 4
        else:
            if visible_posts_number == 3:
                mean_point = (posts[0][1] + posts[1][1] + posts[2][1] ) / 3            # recognition of useless post
                if posts[0][1] < mean_point and posts[1][1] >= mean_point and posts[2][1] >= mean_point \
                                  or posts[0][1] >= mean_point and posts[1][1] < mean_point and posts[2][1] < mean_point : exit_ind = 0
                if posts[1][1] < mean_point and posts[0][1] >= mean_point and posts[2][1] >= mean_point \
                                  or posts[1][1] >= mean_point and posts[0][1] < mean_point and posts[2][1] < mean_point : exit_ind = 1
                if posts[2][1] < mean_point and posts[0][1] >= mean_point and posts[1][1] >= mean_point \
                                  or posts[2][1] >= mean_point and posts[0][1] < mean_point and posts[1][1] < mean_point : exit_ind = 2
                posts.pop(exit_ind)                                                     # deleting of useless post
                visible_posts_number = 2
            if visible_posts_number == 2:
                diff = posts[0][0] - posts[1][0]
                if -1.75 < posts[0][0] < 1.75 and -1.75 < posts[1][0] < 1.75 :
                    if diff > 0: posts[0][2], posts[1][2] = 2 , 1
                    else: posts[0][2], posts[1][2] = 1 , 2
                elif (posts[0][0] > 1.75 or posts[0][0] < -1.75) and (posts[1][0] > 1.75 or posts[1][0] < -1.75):
                    if (diff > 1.75) or ( -1.75 < diff <0 ): posts[0][2], posts[1][2] = 3 , 4
                    else: posts[0][2], posts[1][2] = 4 , 3
            else: return False
        #uprint(posts)
        for i in range(self.post_data_in_pose_number):
        #for raw_post in self.post_data_in_pose:
            #raw_post[2] = 0
            self.post_data_in_pose[i,2] = 0
            for post in posts:
                if abs(post[0] - self.post_data_in_pose[i,0] / 2000)< 0.15 *abs(post[0])\
                        and abs(post[1] - self.post_data_in_pose[i,1] / 2000)< 0.15 *abs(post[1]):
                    self.post_data_in_pose[i,2] = post[2]

        returncode = self.coord_calculation(posts)
        return returncode

    def coord_calculation(self,posts):
        guest_left_yaw, guest_left_dist, guest_right_yaw, guest_right_dist, home_right_yaw, home_right_dist, home_left_yaw, home_left_dist = 0,0,0,0,0,0,0,0
        for i in range(len(posts)):
            if posts[i][2] == 1: guest_right_yaw, guest_right_dist, number = posts[i]
            if posts[i][2] == 2: guest_left_yaw, guest_left_dist, number = posts[i]
            if posts[i][2] == 3: home_right_yaw, home_right_dist, number = posts[i]
            if posts[i][2] == 4:  home_left_yaw, home_left_dist, number = posts[i]
        coord_by_yaw_dist_guest =[]
        coord_by_yaw_yaw_guest =[]
        if not (guest_right_dist == 0 or guest_left_dist == 0) :
            rp = guest_right_yaw
            lp = guest_left_yaw
            #xt = GUEST_GOAL_POST_RIGHT_X - guest_right_dist * math.cos(rp)
            #yt = GUEST_GOAL_POST_RIGHT_Y - guest_right_dist * math.sin(rp)
            xt = self.glob.landmarks['post1'][0][0] - guest_right_dist * math.cos(rp)
            yt = self.glob.landmarks['post1'][0][1] - guest_right_dist * math.sin(rp)
            coord_by_yaw_dist_guest.append([xt,yt])
            #xt = GUEST_GOAL_POST_LEFT_X - guest_left_dist * math.cos(lp)
            #yt = GUEST_GOAL_POST_LEFT_Y - guest_left_dist * math.sin(lp)
            xt = self.glob.landmarks['post2'][0][0] - guest_left_dist * math.cos(lp)
            yt = self.glob.landmarks['post2'][0][1] - guest_left_dist * math.sin(lp)
            coord_by_yaw_dist_guest.append([xt,yt])
            if (math.tan(rp)- math.tan(lp)) != 0:
                #xt = GUEST_GOAL_POST_RIGHT_X + ( GUEST_GOAL_POST_LEFT_Y - GUEST_GOAL_POST_RIGHT_Y) /(math.tan(rp)- math.tan(lp))
                #yt = GUEST_GOAL_POST_RIGHT_Y + ( GUEST_GOAL_POST_LEFT_Y - GUEST_GOAL_POST_RIGHT_Y) * math.tan(rp)/ (math.tan(rp) - math.tan(lp))
                xt = self.glob.landmarks['post1'][0][0]\
                     + ( self.glob.landmarks['post2'][0][1] - self.glob.landmarks['post1'][0][1]) /(math.tan(rp)- math.tan(lp))
                yt = self.glob.landmarks['post1'][0][1]\
                     + ( self.glob.landmarks['post2'][0][1] - self.glob.landmarks['post1'][0][1]) * math.tan(rp)/ (math.tan(rp) - math.tan(lp))
                coord_by_yaw_yaw_guest.append([xt,yt])
            #uprint(' From GUEST GOALS:')
            #uprint( 'МЕТОД 1:')
            #uprint( coord_by_yaw_dist_guest )
            #uprint( 'МЕТОД 2:')
            #uprint(coord_by_yaw_yaw_guest)
        coord_by_yaw_dist_home =[]
        coord_by_yaw_yaw_home =[]
        if not (home_right_dist == 0 or home_left_dist == 0) :
            rp = home_right_yaw
            lp = home_left_yaw
            #xt = HOME_GOAL_POST_RIGHT_X - home_right_dist * math.cos(rp)
            #yt = HOME_GOAL_POST_RIGHT_Y - home_right_dist * math.sin(rp)
            xt = self.glob.landmarks['post3'][0][0] - home_right_dist * math.cos(rp)
            yt = self.glob.landmarks['post3'][0][1] - home_right_dist * math.sin(rp)
            coord_by_yaw_dist_home.append([xt,yt])
            #xt = HOME_GOAL_POST_LEFT_X - home_left_dist * math.cos(lp)
            #yt = HOME_GOAL_POST_LEFT_Y - home_left_dist * math.sin(lp)
            xt = self.glob.landmarks['post4'][0][0] - home_left_dist * math.cos(lp)
            yt = self.glob.landmarks['post4'][0][1] - home_left_dist * math.sin(lp)
            coord_by_yaw_dist_home.append([xt,yt])
            if (math.tan(rp)- math.tan(lp)) != 0:
                #xt = HOME_GOAL_POST_RIGHT_X + ( HOME_GOAL_POST_LEFT_Y - HOME_GOAL_POST_RIGHT_Y) /(math.tan(rp)- math.tan(lp))
                #yt = HOME_GOAL_POST_RIGHT_Y + ( HOME_GOAL_POST_LEFT_Y - HOME_GOAL_POST_RIGHT_Y) * math.tan(rp)/ (math.tan(rp) - math.tan(lp))
                xt = self.glob.landmarks['post3'][0][0]\
                        + ( self.glob.landmarks['post4'][0][1] - self.glob.landmarks['post3'][0][1]) /(math.tan(rp)- math.tan(lp))
                yt = self.glob.landmarks['post3'][0][1]\
                        + ( self.glob.landmarks['post4'][0][1] - self.glob.landmarks['post3'][0][1]) * math.tan(rp)/ (math.tan(rp) - math.tan(lp))
                coord_by_yaw_yaw_home.append( [xt,yt])
            #uprint(' From HOME GOALS:')
            #uprint( 'МЕТОД 1:')
            #uprint( coord_by_yaw_dist_home )
            #uprint( 'МЕТОД 2:')
            #uprint(coord_by_yaw_yaw_home)
        weight = 0
        sum_x, sum_y = 0, 0
        if len(coord_by_yaw_dist_guest) != 0:
            average_dist = (guest_right_dist + guest_left_dist)/2
            if average_dist > 1.6:
                factor = 0.01
                self.quality = 0.01
            else:
                factor = 1
                self.quality = 1
            sum_x = sum_x + (coord_by_yaw_dist_guest[0][0] + coord_by_yaw_dist_guest[1][0]) * factor
            sum_y = sum_y + (coord_by_yaw_dist_guest[0][1] + coord_by_yaw_dist_guest[1][1]) * factor
            weight = weight + 2 * factor
            if len(coord_by_yaw_yaw_guest) != 0:
                sum_x = sum_x + coord_by_yaw_yaw_guest[0][0] * 5 * factor
                sum_y = sum_y + coord_by_yaw_yaw_guest[0][1] * 5 * factor
                weight = weight + 5 * factor
        if len(coord_by_yaw_dist_home) != 0:
            average_dist = (home_right_dist+ home_left_dist)/2
            if average_dist > 1.6:
                factor = 0.01
                self.quality = 0.01
            else:
                factor = 1
                self.quality = 1
            sum_x = sum_x + (coord_by_yaw_dist_home[0][0] + coord_by_yaw_dist_home[1][0]) * factor
            sum_y = sum_y + (coord_by_yaw_dist_home[0][1] + coord_by_yaw_dist_home[1][1]) * factor
            weight = weight + 2 * factor
            if len(coord_by_yaw_yaw_home) != 0:
                sum_x = sum_x + coord_by_yaw_yaw_home[0][0] * 5 * factor
                sum_y = sum_y + coord_by_yaw_yaw_home[0][1] * 5 * factor
                weight = weight + 5 * factor
        if weight == 0: return False
        self.coord_visible = [sum_x/weight, sum_y/weight, self.motion.imu_body_yaw()]
        #uprint('self.coord_visible = ', self.coord_visible)
        return True

    def convert_to_line_vector(self, x1 , y1 , x2 , y2):          # converts lines from (x1,y1),(x2,y2) into [rho,theta]
        if x1 == x2: return x1, 0
        if y1 == y2: return y1, math.pi/2
        theta = math.atan((float(x1) - float(x2))/(float(y2) - float(y1)))
        rho = (float(x1) * float(y2) - float(x2) * float(y1))/(float(x1) - float(x2)) * math.sin(theta)
        return rho, theta

    def detect_line_in_image(self, img):
        if self.glob.SIMULATION == 2 :
            self.sensor.flush()
            img1 = self.sensor.alloc_extra_fb(160, 120, self.sensor.RGB565)
            img2 = self.sensor.alloc_extra_fb(160, 120, self.sensor.RGB565)
            img.copy(x_scale = 0.5, y_scale = 0.5, copy_to_fb = img1)
            img.copy(x_scale = 0.5, y_scale = 0.5, copy_to_fb = img2)
            #img.copy(x_scale = 0.5, y_scale = 0.5, copy_to_fb = img)
            #pyb.delay(1000)
            img4 = img1.binary([self.vision.TH["white marking"]['th']],to_bitmap=True, copy=False)
            img3 = img2.binary([self.vision.TH["green field"]['th']],to_bitmap=True, copy=False)
            
            
            #pyb.delay(1000)
            #point_data = np.zeros((160,2)"", dtype = np.uint8)
            #for x in range(160):
            #    for y in range(120):
            #        #if img3.get_pixel(x,y)[0] > 200: break
            #        if img3.get_pixel(x,y) > 0: break
            #    point_data[x,0] = x
            #    point_data[x,1] = y
            #line_segments_data = self.vision.detect_line_segments( point_data, 160,
            #                                         rank_threshold = 15, line_num_limit = 3 )
            line_segments_data = self.vision.detect_line_segments( img3, rank_threshold = 15,
                                        line_num_limit = 3, upper_lines = True )
            point_data = None
            field_border_data = []
            field_border_segments = []
            for line_segment in line_segments_data:
                rho, theta = self.convert_to_line_vector(line_segment[0], line_segment[1],
                                                    line_segment[2], line_segment[3])
                p_a1, p_b1 = 0, int(rho)
                if theta != 0:
                    p_a1 = -1 / math.tan(theta)
                    p_b1 = rho/math.sin(theta)
                line = []
                if theta == 0: line = [int(rho),0,int(rho),119]
                else:
                    y = int(p_b1)
                    if 0 <= y <= 119 :
                        line.append(0)
                        line.append(y)
                    y = int(159 * p_a1 + p_b1)
                    if 0 <= y <= 119 :
                        line.append(159)
                        line.append(y)
                    if math.cos(theta) != 0:
                        x = int(- p_b1 / p_a1)
                        if 0 <= x <= 159 and len(line) < 4:
                            line.append(x)
                            line.append(0)
                        x = int(- p_b1 / p_a1 + 119 / p_a1)
                        if 0 <= x <= 159 and len(line) < 4:
                            line.append(x)
                            line.append(119)
                #print('line = ', line)
                if len(line) == 4:
                    field_border_segments.append(line)
                    field_border_data.append([p_a1, p_b1])
                    #img.draw_line(line, (255,0,0))
                #print( 'line = ', line)
            #print(img3.compressed_for_ide(), end="")
            #pyb.delay(1000)
            #print(img4.compressed_for_ide(), end="")
            #pyb.delay(1000)
            #print(img.compressed_for_ide(), end="")
            #pyb.delay(1000)
            #print( 'field_border_data = ', field_border_data)
            #print( 'field_border_segments =', field_border_segments)
            len_field_border_data = len(field_border_data)
            if len_field_border_data != 0:
                if len_field_border_data == 3:
                    k = 0
                    for i in range(3):
                        if field_border_segments[i - k] == [0, 119, 159, 119]:
                            field_border_data.pop(i - k)
                            field_border_segments.pop(i - k)
                            len_field_border_data -= 1
                            k += 1
                    if k == 0:
                        cross_points_num = 0
                        a0, b0 = field_border_data[0][0], field_border_data[0][1]
                        a1, b1 = field_border_data[1][0], field_border_data[1][1]
                        a2, b2 = field_border_data[2][0], field_border_data[2][1]
                        if a0 != a1:
                            cross_point01 = [int((b0 - b1)/(a1 - a0)), int(a0 * (b0 - b1)/(a1 - a0) + b0)]
                            if 0 <= cross_point01[0] < 160 and 0 <= cross_point01[1] < 120:
                                cross_points_num += 1
                            else: cross_point01 = None
                        else: cross_point01 = None
                        if a1 != a2:
                            cross_point12 = [int((b1 - b2)/(a2 - a1)), int(a1 * (b1 - b2)/(a2 - a1) + b1)]
                            if 0 <= cross_point12[0] < 160 and 0 <= cross_point12[1] < 120:
                                cross_points_num += 1
                            else: cross_point12 = None
                        else: cross_point12 = None
                        if a0 != a2:
                            cross_point02 = [int((b0 - b2)/(a2 - a0)), int(a0 * (b0 - b2)/(a2 - a0) + b0)]
                            if 0 <= cross_point02[0] < 160 and 0 <= cross_point02[1] < 120:
                                cross_points_num += 1
                            else: cross_point02 = None
                        else: cross_point02 = None
                        if cross_points_num == 3:
                            field_border_data.pop(2)
                            field_border_segments.pop(2)
                            len_field_border_data = 2
                if len_field_border_data == 2:
                    k = 0
                    for i in range(2):
                        if field_border_segments[i - k] == [0, 119, 159, 119]:
                            field_border_data.pop(i - k)
                            field_border_segments.pop(i - k)
                            len_field_border_data -= 1
                            k += 1
                if len_field_border_data == 2:
                    a0, b0 = field_border_data[0][0], field_border_data[0][1]
                    a1, b1 = field_border_data[1][0], field_border_data[1][1]
                    if round(a0, 2) == 0:
                        if (a1 > 0 and field_border_segments[0][0] > field_border_segments[1][0]) or\
                           (a1 < 0 and field_border_segments[0][2] < field_border_segments[1][2]) or\
                           (field_border_segments[0][0] == 0 and field_border_segments[0][2] == 159) :
                            field_border_data.pop(1)
                            field_border_segments.pop(1)
                for x in range(160):
                    border_y_candidate = []
                    for parameter in field_border_data:
                        border_y_candidate.append(parameter[0] * x + parameter[1])
                    border_y = max(border_y_candidate)
                    for y in range(120):
                        if y < border_y:
                            img4.set_pixel(x, y, 0)
            for line in field_border_segments: img.draw_line([line[0] * 2, line[1] * 2, line[2] * 2, line[3] * 2], (255,0,0))
            img4.zhangSuen()
            #self.vision.thinning(img4, 120, 160, 100)
            #print(img4.compressed_for_ide(), end="")
            #pyb.delay(1000)
            #point_data = self.glob.line_alloc
            #data_size = 0
            #for x in range(160):
            #    for y in range(120):
            #        if img4.get_pixel(x,y) > 0:
            #            if data_size >= 2000: break
            #            point_data[data_size,0] = x
            #            point_data[data_size,1] = y
            #            data_size += 1
            line_segments = self.vision.detect_line_segments( img4 )
            line_segments_data = []
            for line in line_segments:
                line_new = [line[0]*2, line[1]*2, line[2]*2, line[3]*2]
                line_segments_data.append(line_new)
                img.draw_line(line_new, (255,0,0))
                uprint( 'line_segment = ', line)
            #print(img.compressed_for_ide(), end="")
            self.sensor.flush()
            self.sensor.dealloc_extra_fb()
            self.sensor.dealloc_extra_fb()
        else:                                                                   # simulation
            img1 = self.re.Image(img)
            #self.cv2.imshow('Vision Binary', img1.img)
            #self.cv2.waitKey(0) & 0xFF
            img_white = img1.binary(self.vision.TH["white marking"]['th'])    # detect white color areas
            #print("img_white type", img_white.dtype)
            #show_mask_and_annotated_objects(img1.img, "lala", img_white)#, [((100, 200), (300, 500))])

            img2 = self.cv2.resize(img1.img, (160, 120))
            output_img = cv2.cvtColor(img2, cv2.COLOR_LAB2BGR)
            #self.cv2.imshow('marking', img_white)
            #self.cv2.waitKey(0) & 0xFF
            img2r = self.re.Image(img2)
            #cv2.imshow('source', img2r.img)
            #self.cv2.waitKey(1) & 0xFF

            green_mask = img2r.binary(self.vision.TH["green field"]['th'])             # detect green color areas
            #print("w ",self.vision.TH["white marking"]['th'])
            #print("w ",self.vision.TH["green field"]['th'])

            #green_mask = cv2.bitwise_not(green_mask)
            #print(green_mask.shape)
            #print(np.sum(green_mask)/green_mask.size)
            #self.cv2.imshow('green filter', green_mask)
            #self.cv2.waitKey(1) & 0xFF

            line_segments_data = self.vision.detect_line_segments( green_mask, rank_threshold = 8,
                                        line_num_limit = 3, upper_lines = True )
            #print("Upper lines: ", line_segments_data)
            point_data = None
            field_border_data = []
            field_border_segments = []
            for line_segment in line_segments_data:
                l = line_segment
                cv2.line(output_img, (l[0], l[1]), (l[2], l[3]), (130, 190, 230), 3)

                rho, theta = self.convert_to_line_vector(line_segment[0], line_segment[1],
                                                    line_segment[2], line_segment[3])
                p_a1, p_b1 = 0, int(rho)
                if theta != 0:
                    p_a1 = -1 / math.tan(theta)
                    p_b1 = rho/math.sin(theta)
                line = []
                if theta == 0: line = [int(rho),0,int(rho),119]
                else:
                    y = int(p_b1)
                    if 0 <= y <= 119 :
                        line.append(0)
                        line.append(y)
                    y = int(159 * p_a1 + p_b1)
                    if 0 <= y <= 119 :
                        line.append(159)
                        line.append(y)
                    if math.cos(theta) != 0:
                        x = int(- p_b1 / p_a1)
                        if 0 <= x <= 159 and len(line) < 4:
                            line.append(x)
                            line.append(0)
                        x = int(- p_b1 / p_a1 + 119 / p_a1)
                        if 0 <= x <= 159 and len(line) < 4:
                            line.append(x)
                            line.append(119)
                #print('line = ', line)
                if len(line) == 4:
                    field_border_segments.append(line)
                    field_border_data.append([p_a1, p_b1])

            #cv2.imshow("line segments", img2)
            #print("segments", field_border_segments, " data: ", field_border_data)
            len_field_border_data = len(field_border_data)
            if len_field_border_data != 0:
                if len_field_border_data == 3:
                    k = 0
                    for i in range(3):
                        if field_border_segments[i - k] == [0, 119, 159, 119]:
                            field_border_data.pop(i - k)
                            field_border_segments.pop(i - k)
                            len_field_border_data -= 1
                            k += 1
                    if k == 0:
                        cross_points_num = 0
                        a0, b0 = field_border_data[0][0], field_border_data[0][1]
                        a1, b1 = field_border_data[1][0], field_border_data[1][1]
                        a2, b2 = field_border_data[2][0], field_border_data[2][1]
                        if a0 != a1:
                            cross_point01 = [int((b0 - b1)/(a1 - a0)), int(a0 * (b0 - b1)/(a1 - a0) + b0)]
                            if 0 <= cross_point01[0] < 160 and 0 <= cross_point01[1] < 120:
                                cross_points_num += 1
                            else: cross_point01 = None
                        else: cross_point01 = None
                        if a1 != a2:
                            cross_point12 = [int((b1 - b2)/(a2 - a1)), int(a1 * (b1 - b2)/(a2 - a1) + b1)]
                            if 0 <= cross_point12[0] < 160 and 0 <= cross_point12[1] < 120:
                                cross_points_num += 1
                            else: cross_point12 = None
                        else: cross_point12 = None
                        if a0 != a2:
                            cross_point02 = [int((b0 - b2)/(a2 - a0)), int(a0 * (b0 - b2)/(a2 - a0) + b0)]
                            if 0 <= cross_point02[0] < 160 and 0 <= cross_point02[1] < 120:
                                cross_points_num += 1
                            else: cross_point02 = None
                        else: cross_point02 = None
                        if cross_points_num == 3:
                            field_border_data.pop(2)
                            field_border_segments.pop(2)
                            len_field_border_data = 2
                if len_field_border_data == 2:
                    k = 0
                    for i in range(2):
                        if field_border_segments[i - k] == [0, 119, 159, 119]:
                            field_border_data.pop(i - k)
                            field_border_segments.pop(i - k)
                            len_field_border_data -= 1
                            k += 1
                if len_field_border_data == 2:
                    a0, b0 = field_border_data[0][0], field_border_data[0][1]
                    a1, b1 = field_border_data[1][0], field_border_data[1][1]
                    if round(a0, 2) == 0:
                        if (a1 > 0 and field_border_segments[0][0] > field_border_segments[1][0]) or\
                           (a1 < 0 and field_border_segments[0][2] < field_border_segments[1][2]) or\
                           (field_border_segments[0][0] == 0 and field_border_segments[0][2] == 159) :
                            field_border_data.pop(1)
                            field_border_segments.pop(1)

                for x in range(320):
                    border_y_candidate = []
                    for parameter in field_border_data:
                        border_y_candidate.append(parameter[0] * x + parameter[1] * 2)
                    border_y = max(border_y_candidate)
                    #print(border_y)
                    for y in range(240):
                        if y < border_y:
                            img_white[y][x] = [0,0,0]
                        else: break
            img_w_g = self.copy.deepcopy(img_white)
            #for x in range(320):                                                #| set to black area over green field
            #    for y in range (240):                                           #|
            #        if green_mask[y][x].all() == 0: img_w_g[y][x] = [0,0,0]     #|
            #        else: break                                                 #|
            #self.cv2.imshow('img_w_g', img_w_g)
            #self.cv2.waitKey(0) & 0xFF
            kernel = self.np.ones((5,5), self.np.uint8)
            img_w_g = self.cv2.dilate(img_w_g,kernel,iterations = 1)
            img_w_g = self.cv2.GaussianBlur(img_w_g,(5,5),5)
            #img_w_g = self.cv2.dilate(img_w_g,kernel,iterations = 1)
            #img_w_g = self.cv2.erode(img_w_g,kernel,iterations = 2)
            #img_w_g = self.cv2.dilate(img_w_g,kernel,iterations = 1)
            img_w_g= self.cv2.ximgproc.thinning(self.cv2.cvtColor(img_w_g, self.cv2.COLOR_BGR2GRAY))
            img3  = self.re.Image(img_w_g)
            #cv2.imshow('img_w_g', cv2.resize(img3.img, (160, 120)))
            #self.cv2.waitKey(0) & 0xFF
            #lines = img3.find_line_segments()                           # raw line segments are detected in image
            lines = self.vision.detect_line_segments(cv2.resize(img3.img, (160, 120)), rank_threshold = 8,
                                        line_num_limit = 5, upper_lines = False )
            #print("Not upper lines: ", lines)
            img2 = self.cv2.resize(img1.img, (160, 120))
            relo = self.re.Image(cv2.cvtColor(img2, cv2.COLOR_LAB2BGR))

            line_segments_data =[]
            for j in range(len(lines)):
                l = lines[j]
                relo.draw_line(self.re.Line(l[0], l[1], l[2], l[3], 0))
                line_segments_data.append([comp * 2 for comp in l])
                        
            #img_for_lines = self.copy.deepcopy(img1.img)

            #show_mask_and_annotated_objects(img_for_lines, "lines", img_white, line_segments_data, "lines")
            show_mask_and_annotated_objects(output_img, "lines", relo.img, line_segments_data, "lines")

            #self.cv2.imshow('Vision Binary', img3.img)
            self.cv2.waitKey(1) & 0xFF

        #self.floor_lines = []
        self.field_border_data = field_border_data
        for line in line_segments_data:
            x1,y1,x2,y2 = line
            weight = abs(x1 - x2) + abs(y1 - y2)
            returncode, floor_x1, floor_y1 = self.motion.get_cooord_of_point(x1, y1)
            if returncode == False or self.max_field_dimension < math.sqrt(floor_x1**2 + floor_y1**2): continue
            returncode, floor_x2, floor_y2 = self.motion.get_cooord_of_point(x2, y2)
            if returncode == False or self.max_field_dimension < math.sqrt(floor_x2**2 + floor_y2**2): continue
            #line_length =  math.sqrt((floor_x1 - floor_x2)**2+ (floor_y1 - floor_y2)**2)    # check length of segment on floor
            rho, theta = self.convert_to_line_vector(floor_x1, floor_y1, floor_x2, floor_y2)
            #print( 'floor_x1 = ', int(floor_x1*1000),'\t', 'floor_y1 =', int(floor_y1*1000),'\t', 'floor_x2 =', int(floor_x2*1000),'\t', 'floor_y2 =', int(floor_y2*1000) )
            self.floor_lines.append([rho,theta, floor_x1, floor_y1, floor_x2, floor_y2, weight])
        return

    def group_lines_find_crosses(self):
        line_group =[]
        for i in range(len(self.floor_lines)):
            if line_group == []:
                line_group.append([self.floor_lines[0]])
                continue
            for j in range(len(line_group)):
                if abs(self.floor_lines[i][0] - line_group[j][0][0]) < 0.4 and abs(self.floor_lines[i][1] - line_group[j][0][1]) < 0.35 :
                    line_group[j].append(self.floor_lines[i])
                    flag = 1
                    break
                flag = 0
            if flag == 0:
                line_group.append([self.floor_lines[i]])
        line_group_compact = []
        for i in range(len(line_group)):
            rho, theta = 0, 0
            min_x1 = min(line_group[i][0][2], line_group[i][0][4])
            max_x2 = max(line_group[i][0][2], line_group[i][0][4])
            min_y1 = min(line_group[i][0][3], line_group[i][0][5])
            max_y2 = max(line_group[i][0][3], line_group[i][0][5])
            n = len(line_group[i])
            total_weight = 0
            for j in range(n):
                rho += line_group[i][j][0] * line_group[i][j][6]
                theta += line_group[i][j][1] * line_group[i][j][6]
                total_weight += line_group[i][j][6]
                #line_length += line_group[i][j][2]
                minmax = min(line_group[i][j][2], line_group[i][j][4])
                if minmax < min_x1 : min_x1 = minmax
                minmax = max(line_group[i][j][2], line_group[i][j][4])
                if minmax > max_x2 : max_x2 = minmax
                minmax = min(line_group[i][j][3], line_group[i][j][5])
                if minmax < min_y1 : min_y1 = minmax
                minmax = max(line_group[i][j][3], line_group[i][j][5])
                if minmax > max_y2 : max_y2 = minmax
            line_length_square = (max_x2-min_x1)**2+(max_y2-min_y1)**2
            if line_length_square > 1.0 :                                # add only if visible length of line is big enougth
                weight = 1
                #if rho/total_weight > 2.5: weight = 0.5
                line_group_compact.append([rho/total_weight,theta/total_weight, weight])
                #print('min_x1 = ', min_x1, '\t', 'max_x2 = ', max_x2, '\t', 'min_y1 = ', min_y1, '\t', 'max_y2 = ', max_y2  )
        self.cross_points =[]
        n = len(line_group_compact)
        if  n > 1:
            for i in range(n):
                for j in range(i+1,n):
                    theta_spread = abs(line_group_compact[i][1] - line_group_compact[j][1])
                    if theta_spread > 2*math.pi: theta_spread %= (2 * math.pi)
                    if theta_spread > math.pi: theta_spread = (2 * math.pi) - theta_spread
                    if  0.43 < theta_spread < 2.62 :
                        r1, t1 = line_group_compact[i][0], line_group_compact[i][1]
                        r2, t2 = line_group_compact[j][0], line_group_compact[j][1]
                        alpha = math.atan((r1/r2*math.cos(t2) -math.cos(t1))/(math.sin(t1) -r1/r2*math.sin(t2)))
                        D = r1/math.cos(alpha -t1)
                        if D < self.max_field_dimension:
                            cross_x = D * math.cos(alpha)
                            cross_y = D * math.sin(alpha)
                            weight = 1
                            #if D*D > 6.25:weight = 0.5
                            self.cross_points.append([cross_x, cross_y, weight])
        self.line_group_compact.clear()
        self.line_group_compact = line_group_compact
        self.floor_lines.clear()
        return

    def detect_penalty_marks(self, img):
        if self.glob.SIMULATION == 2 :
            img1 = img
        else: 
            img1 = self.re.Image(img)
            labimg = img
            #labimg = self.cv2.cvtColor (img, self.cv2.COLOR_BGR2LAB)
        penalty_marks_candidate = []
        #if self.glob.SIMULATION == 2 :
        thresholds =  [self.vision.TH['white marking']['th']]
        #else: thresholds =  self.vision.TH['white marking']['th']
        for blob in img1.find_blobs(thresholds, pixels_threshold=self.vision.TH['white marking']['pixel'],
                                   area_threshold=self.vision.TH['white marking']['area'], merge=True):
            if blob.x() < 5 or blob.y() < 5 or blob.x() + blob.w() > 314 or blob.y() + blob.h() > 234 : continue
            n = 0
            per_R, per_G, per_B = 0.0,0.0,0.0
            if self.glob.SIMULATION == 2 :
                for x in range((blob.x()-5), (blob.x()+blob.w()+5)):
                    a=self.image.rgb_to_lab(img1.get_pixel(x , blob.y()-5))
                    b=self.image.rgb_to_lab(img1.get_pixel(x , blob.y()+ blob.h()+ 5))
                    per_R += a[0] + b[0]
                    per_G += a[1] + b[1]
                    per_B += a[2] + b[2]
                    n += 2
                for y in range((blob.y() - 5), (blob.y()+ blob.h()+ 5)):
                    a=self.image.rgb_to_lab(img1.get_pixel(blob.x()-5 , y))
                    b=self.image.rgb_to_lab(img1.get_pixel(blob.x()+ blob.w()+ 5, y))
                    per_R += a[0] + b[0]
                    per_G += a[1] + b[1]
                    per_B += a[2] + b[2]
                    n += 2
                per = [per_R/n, per_G/n, per_B/n]
                is_green = (self.vision.TH['green field']['th'][0] < per[0] < self.vision.TH['green field']['th'][1]) and \
                           (self.vision.TH['green field']['th'][2] < per[1] < self.vision.TH['green field']['th'][3]) and \
                           (self.vision.TH['green field']['th'][4] < per[2] < self.vision.TH['green field']['th'][5])
            else:
                for x in range((blob.x()-5), (blob.x()+blob.w()+5)):
                    per_R += labimg[blob.y()-5][x][0]
                    per_G += labimg[blob.y()-5][x][1]
                    per_B += labimg[blob.y()-5][x][2]
                    per_R +=  labimg[blob.y()+ blob.h()+ 5][x][0]
                    per_G +=  labimg[blob.y()+ blob.h()+ 5][x][1]
                    per_B +=  labimg[blob.y()+ blob.h()+ 5][x][2]
                    n += 2
                for y in range((blob.y() - 5), (blob.y()+ blob.h()+ 5)):
                    per_R += labimg[y][blob.x()-5][0]
                    per_G += labimg[y][blob.x()-5][1]
                    per_B += labimg[y][blob.x()-5][2]
                    per_R +=  labimg[y][blob.x()+ blob.w()+ 5][0]
                    per_G +=  labimg[y][blob.x()+ blob.w()+ 5][1]
                    per_B +=  labimg[y][blob.x()+ blob.w()+ 5][2]
                    n += 2
                per = [per_R/n, per_G/n, per_B/n]
                is_green = (self.vision.TH['green field']['th'][0] < per[0]/ 2.55 <  self.vision.TH['green field']['th'][1])\
                       and (self.vision.TH['green field']['th'][2] < per[1]- 128 <  self.vision.TH['green field']['th'][3])\
                       and (self.vision.TH['green field']['th'][4] < per[2]- 128 <  self.vision.TH['green field']['th'][5])
            if is_green == True: penalty_marks_candidate.append(blob)
        penalty_marks = []
        for i in range(len(penalty_marks_candidate)):
            if penalty_marks_candidate[i].w() * 0.35 < penalty_marks_candidate[i].h() < penalty_marks_candidate[i].w() * 1.2 :
                penalty_marks.append(penalty_marks_candidate[i])
        for i in range(len(penalty_marks)):
            img1.draw_rectangle(penalty_marks[i].rect())
        for i in range (len(penalty_marks)):
            returncode, floor_x, floor_y = self.motion.get_cooord_of_point(penalty_marks[i].cx(), penalty_marks[i].cy())
            if returncode == False: continue
            weight = 1
            #if floor_x**2 + floor_y**2 > 6.25: weight = 0.5
            self.penalty.append([floor_x, floor_y, weight])
        #uprint(penalty_marks)
        #self.cv2.imshow('Vision Binary', img1.img)
        #self.cv2.waitKey(0) & 0xFF

    def group_penalty_marks(self):
        self.penalty_points.clear()
        penalty = []
        m = len(self.penalty)
        for i in range(m):
            if i == 0:
                penalty.append([self.penalty[i]])
            else:
                was_not_added = True
                for j in range(len(penalty)):
                    if (self.penalty[i][0] - penalty[j][0][0])**2 + (self.penalty[i][1] - penalty[j][0][1])**2 < 0.36 :
                        penalty[j].append(self.penalty[i])
                        was_not_added = False
                if was_not_added == True: penalty.append([self.penalty[i]])
        for i in range(len(penalty)):
            n = len(penalty[i])
            x, y, w = 0, 0, 0
            for j in range(n):
                x += penalty[i][j][0]
                y += penalty[i][j][1]
                w += penalty[i][j][2]
            if n != 0 : self.penalty_points.append([x/n,y/n,w/n])
        self.penalty.clear()

    def detect_obstacles(self, img):
        if self.glob.SIMULATION == 2 :
            if abs(self.motion.neck_pan) == 2667 and self.motion.neck_tilt == self.motion.neck_play_pose -1400: return
            img1 = img
        else:
            img1 = self.re.Image(img)
        #green_mask = img1.binary(self.vision.TH["green field"]['th'])             # detect green color areas
        if len(self.field_border_data) != 0:
            for x in range(320):
                border_y_candidate = []
                for parameter in self.field_border_data:
                    border_y_candidate.append(parameter[0] * x + parameter[1] * 2)
                border_y = max(border_y_candidate)
                if self.glob.SIMULATION == 2 :
                    for y in range(240):
                        if y < border_y:
                            img1.set_pixel(x,y,(0,0,0))
                else:
                    for y in range(240):
                        if y < border_y:
                            img1.img[y][x] = [0,0,0]
                    else: break
        thresholds =  [self.vision.TH['orange ball']['th'],
                       self.vision.TH['blue posts']['th'],
                       self.vision.TH['yellow posts']['th'],
                       self.vision.TH['green field']['th'],
                       self.vision.TH['white marking']['th'],
                       [0,0,0,0,0,0]]
        if self.glob.SIMULATION == 2 :
            roi1 = (0,0,320,240)
            if (abs(self.motion.neck_pan) == 2667 and self.motion.neck_tilt == self.motion.neck_play_pose -700) or\
               (abs(self.motion.neck_pan) == 1333 and self.motion.neck_tilt == self.motion.neck_play_pose -1400):
                roi1 = (0,0,320,160)
            blobs = img1.find_blobs(thresholds, roi = roi1, invert = True, pixels_threshold=self.vision.TH['blue posts']['pixel'],
                                   area_threshold=self.vision.TH['blue posts']['area'], merge=True)

        else:
            if self.motion.neck_pan == 0 and self.motion.neck_tilt == self.motion.neck_play_pose -1400:
                img1.img[223:][:] = [0,0,0]
                for i in range(208,223):
                    img1.img[i][80:235] = [0,0,0]
            if self.motion.neck_pan == -1333 and self.motion.neck_tilt == self.motion.neck_play_pose -1400:
                for i in range(92,240):
                    img1.img[i][:135] = [0,0,0]
                for i in range(127,240):
                    img1.img[i][135:i-127+135] = [0,0,0]
            if self.motion.neck_pan == -1333 and self.motion.neck_tilt == self.motion.neck_play_pose -700:
                for i in range(170,240):
                    img1.img[i][:math.ceil((i-169)/70*50)] = [0,0,0]
            if self.motion.neck_pan == 1333 and self.motion.neck_tilt == self.motion.neck_play_pose -700:
                for i in range(170,240):
                    img1.img[i][-math.ceil((i-169)/70*50):] = [0,0,0]
            if self.motion.neck_pan == 1333 and self.motion.neck_tilt == self.motion.neck_play_pose -1400:
                for i in range(92,240):
                    img1.img[i][-135:] = [0,0,0]
                for i in range(127,240):
                    img1.img[i][-(i-127+140):-135] = [0,0,0]
            if self.motion.neck_pan == 2667 and self.motion.neck_tilt == self.motion.neck_play_pose -1400:
                img1.img[185:][:] = [0,0,0]
                for i in range(240):
                    img1.img[i][50:275] = [0,0,0]
                    if i > 110:
                        img1.img[i][275:] = [0,0,0]
            if self.motion.neck_pan == 2667 and self.motion.neck_tilt == self.motion.neck_play_pose -700:
                for i in range(120,240):
                    img1.img[i][70:245] = [0,0,0]
                    if i > 224:
                        img1.img[i][245:] = [0,0,0]
            if self.motion.neck_pan == -2667 and self.motion.neck_tilt == self.motion.neck_play_pose -1400:
                img1.img[185:][:] = [0,0,0]
                for i in range(240):
                    img1.img[i][40:270] = [0,0,0]
                    if i > 110:
                        img1.img[i][:40] = [0,0,0]
            if self.motion.neck_pan == -2667 and self.motion.neck_tilt == self.motion.neck_play_pose -700:
                for i in range(120,240):
                    img1.img[i][75:250] = [0,0,0]
                    if i > 224:
                        img1.img[i][:75] = [0,0,0]
            blobs = img1.find_blobs(thresholds, pixels_threshold=self.vision.TH['blue posts']['pixel'],
                                   area_threshold=self.vision.TH['blue posts']['area'], merge=True, invert = True)
        obstacle_blobs = []
        for blob in blobs:
            #img1.draw_rectangle(blob.rect())
            #uprint('neck_pan = ', self.motion.neck_pan, 'neck_tilt = ', self.motion.neck_tilt, 'blob_rect =', blob.rect())
            obstacle_blobs.append(blob)
        merged_blobs = self.merge_blobs(obstacle_blobs, th=15 )
        for blob in merged_blobs:
            if blob.y() + blob.h() >= 239: continue
            returncode, floor_xc, floor_yc = self.motion.get_cooord_of_point(blob.cx(), blob.y()+blob.h())
            returncode, floor_x1, floor_y1 = self.motion.get_cooord_of_point(blob.x(), blob.y()+blob.h())
            obstacle_diameter = math.sqrt((floor_xc-floor_x1)**2 + (floor_yc-floor_y1)**2) * 2
            dist = math.sqrt((floor_xc)**2 + (floor_yc)**2)
            floor_x = floor_xc * (dist + obstacle_diameter/2)/dist * 1.05
            floor_y = floor_yc * (dist + obstacle_diameter/2)/dist * 1.05
            self.glob.obstacles.append([floor_x, floor_y, obstacle_diameter])
        if len(merged_blobs) >0 : img1.draw_rectangle(merged_blobs[0].rect(), color = (0,255,255))
        if self.glob.SIMULATION == 2 :
            print(img1.compressed_for_ide(), end="")
        else:
            self.cv2.imshow('Obstacle View', img1.img)
            self.cv2.waitKey(10) & 0xFF
            #self.cv2.waitKey(0) & 0xFF

    def merge_blobs(self, blobs_, th=2):
        blobs = []
        for bl in blobs_:
            blobs.append(M_blob(bl.x(), bl.y(), bl.w(), bl.h()))
        if len(blobs) <= 1: return blobs
        merged_blobs = []
        bottoms = []
        for i in range(len(blobs)):
            bottoms.append(blobs[i].y()+blobs[i].h())
        primary = bottoms.index(max(bottoms))
        merged_blobs.append(blobs[primary])
        for i in range(len(blobs)):
            merge = False
            merge_x = False
            merge_y = False
            if blobs[i].x() < blobs[primary].x():
                if blobs[i].x()+blobs[i].w() + th > blobs[primary].x() :  merge_x = True
            else:
                if blobs[primary].x()+blobs[primary].w() + th > blobs[i].x() :  merge_x = True
            if blobs[i].y() < blobs[primary].y():
                if blobs[i].y()+blobs[i].h() + th > blobs[primary].y() :  merge_y = True
            else:
                if blobs[primary].y()+blobs[primary].h() + th > blobs[i].y() :  merge_y = True
            if merge_y and merge_x: merge = True
            if merge:
                if (blobs[primary].y()+blobs[primary].h()) - (blobs[i].y()+blobs[i].h()) <\
                    max(abs(blobs[primary].x()-blobs[i].x()),
                        abs(blobs[primary].x()-blobs[i].x()+ blobs[primary].w()-blobs[i].w())):
                    blob_x = min(blobs[primary].x(), blobs[i].x())
                    blob_w = max((blobs[primary].x()+blobs[primary].w()), (blobs[i].x() + blobs[i].w())) - blob_x
                    blob_y = min(blobs[primary].y(), blobs[i].y())
                    blob_h = max((blobs[primary].y()+blobs[primary].h()), (blobs[i].y() + blobs[i].h())) - blob_y
                    blob = M_blob(blob_x, blob_y, blob_w, blob_h)
                    merged_blobs.pop(0)
                    merged_blobs.insert(0, blob)
            else:
                merged_blobs.append(blobs[i])
        return merged_blobs

    def group_obstacles(self):
        grouped_obstacles = []
        #uprint('obstacles(raw): ', self.glob.obstacles)
        while(len(self.glob.obstacles) > 0):
            obstacle0 = self.glob.obstacles.pop(0)
            group_number = 1
            k = 0
            for i in range(len(self.glob.obstacles)):
                united_obstacles = math.sqrt((obstacle0[0]-self.glob.obstacles[i-k][0])**2 + (obstacle0[1]-self.glob.obstacles[i-k][1])**2)\
                                               < (obstacle0[2] + self.glob.obstacles[i-k][2])/2
                if united_obstacles:
                    group_number += 1
                    new_size = math.sqrt((obstacle0[0]-self.glob.obstacles[i-k][0])**2 + (obstacle0[1]-self.glob.obstacles[i-k][1])**2)\
                                               + (obstacle0[2] + self.glob.obstacles[i-k][2])/2
                    obstacle0 = [(obstacle0[0]*(group_number-1) + self.glob.obstacles[i-k][0])/group_number,
                                 (obstacle0[1]*(group_number-1) + self.glob.obstacles[i-k][1])/group_number,
                                 (obstacle0[2]*(group_number-1) + new_size)/group_number,]
                    self.glob.obstacles.pop(i-k)
                    k += 1
            if obstacle0[2] > 0.1:
                grouped_obstacles.append(obstacle0)
        self.glob.obstacles = []
        for obstacle in grouped_obstacles:
            global_x = self.glob.pf_coord[0] + obstacle[0] * math.cos(self.glob.pf_coord[2]) - obstacle[1] * math.sin(self.glob.pf_coord[2])
            global_y = self.glob.pf_coord[1] + obstacle[1] * math.cos(self.glob.pf_coord[2]) + obstacle[0] * math.sin(self.glob.pf_coord[2])
            if abs(global_y) <= self.glob.landmarks['FIELD_WIDTH']/2 and abs(global_x) <= self.glob.landmarks['FIELD_LENGTH']/2:
                obstacle[0] = global_x
                obstacle[1] = global_y
                self.glob.obstacles.append(obstacle)
        uprint('obstacles: ', self.glob.obstacles)





if __name__=="__main__":
    print('This is not main module!')









