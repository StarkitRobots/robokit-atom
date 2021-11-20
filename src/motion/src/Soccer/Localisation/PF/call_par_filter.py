
#import sys, os

#current_work_directory = os.getcwd()
#current_work_directory = current_work_directory.replace('\\', '/')
#if sys.version != '3.4.0':
    #current_work_directory += '/'

#sys.path.append( current_work_directory + 'Soccer/')
#sys.path.append( current_work_directory + 'Soccer/Motion/')
#sys.path.append( current_work_directory + 'Soccer/Vision/')
#sys.path.append( current_work_directory + 'Soccer/Localisation/')
#sys.path.append( current_work_directory + 'Soccer/Localisation/PF/')



#import math
#import json

from ParticleFilter import ParticleFilter, Robot

class Call_Par_Filter:
    def __init__(self, glob, coord):
        #self.ballPosSelf = None
        #self.ball_position = None
        self.localized = False
        #self.seeBall = False
        #with open(current_work_directory + "Soccer/Localisation/PF/landmarks.json", "r") as f:
        #    landmarks = json.loads(f.read())
        #self.pf = ParticleFilter(Robot(coord[0], coord[1], coord[2]), Field(
            #current_work_directory + "Soccer/Localisation/PF/parfield.json"), landmarks, n = 1000)
        self.pf = ParticleFilter(Robot(coord[0], coord[1], coord[2]), glob)

        #self.robot_position =(x, y, yaw)        #(pf.x, pf.y, pf.yaw)

    def update(self, data, other_coord):                                    # data: {'yellow_posts': [[metr, metr, weight],...,[metr,metr, weight]], 'blue_posts': [[metr, metr, weight],...,[metr,metr, weight]]}
        self.robot_position = self.pf.updatePF( data, other_coord)          # other_coord:  [metr,metr,radians] or []
        if self.pf.consistency > 0.5:                                       #{'Line_crosses': [[metr, metr, weight],...,[metr,metr, weight]]}
            self.localized = True                                           #{'lines': [[rho,theta,weight],..]}
        else:
            self.localized = False                                          #{'penalty': [[metr, metr, weight],...,[metr,metr, weight]]}

    def move(self, odometry):               # odometry:  {'shift_x': Metr, 'shift_y': Metr, 'shift_yaw': Radian}
        self.pf.particles_move(odometry)

    def return_coord(self):                 # (metr,metr)
        return self.pf.return_coord()

    def update_coord(self):
        self.pf.update_coord(self.pf.p)




if __name__=="__main__":
    print('This is not main module!')
