import json

class Vision:
    def __init__(self, glob):
        self.glob = glob
        if self.glob.SIMULATION == 2: 
            with open(self.glob.current_work_directory + "Init_params/Real/Real_Thresholds.json", "r") as f:
                self.TH = json.loads(f.read())
            #from zhangSuen_Thinning import zhangSuen
            from ransac_line_segments import build_line_segments
            #self.zhangSuen = zhangSuen
            self.build_line_segments = build_line_segments
        elif  self.glob.SIMULATION == 5:
            with open(self.glob.current_work_directory + "Init_params/Real/Real_Thresholds.json", "r") as f:
                self.TH = json.loads(f.read())
            from ransac_line_segments_simulation import build_line_segments
            self.build_line_segments = build_line_segments
               
        else:
            with open(self.glob.current_work_directory + "Init_params/Sim/Sim_Thresholds.json", "r") as f:
                self.TH = json.loads(f.read())
            from ransac_line_segments_simulation import build_line_segments
            self.build_line_segments = build_line_segments

    def thinning(self, img, rows, columns, iterations_limit):
        return self.zhangSuen(img, rows, columns, iterations_limit)

    def detect_line_segments( self, img, rank_threshold = 30, line_num_limit = 5, upper_lines = False):
        if self.glob.SIMULATION == 2:
            if upper_lines == True: upperlines = 1
            else: upperlines = 0
            deviation = 2
            number_of_iterations = 10
            raw_lines = img.ransac_segments(rank_threshold, line_num_limit,
                                    upperlines, deviation , number_of_iterations)
            line_segments = []
            for i in range(0,len(raw_lines),4):
                line_segments.append([raw_lines[i],raw_lines[i+1],raw_lines[i+2],raw_lines[i+3]])
        else:
            line_segments = self.build_line_segments( img, rank_threshold, line_num_limit, upper_lines )
        return line_segments



if __name__=="__main__":
    v = Vision(1)
    print(v.TH['orange ball'])

