import numpy as np
import cv2
from common.module import Module
from .image_processing import *

class Colorspace_transform(Module):
    transforms = {"RGB2BGR": cv2.COLOR_RGB2BGR,
                       "RGB2GRAY": cv2.COLOR_RGB2GRAY,
                       "GRAY2RGB": cv2.COLOR_GRAY2RGB,
                       "RGB2HSV": cv2.COLOR_RGB2HSV,
                       "HSV2RGB": cv2.COLOR_HSV2RGB,
                       "RGB2YCrCb": cv2.COLOR_RGB2YCrCb,
                       "YCrCb2RGB": cv2.COLOR_YCrCb2RGB,
                       }

    def __init__(self, params):
        if "show" in params.keys():
            self.show = params["show"]
        else:
            self.show = False
        
        super().__init__(params, "Colorspace_transform", self.show)

        self.result = np.zeros((3, 3, 3), np.uint8)

        self.source_colorspace = params["from"]
        self.target_colorspace = params["to"]

    def apply(self, input):
        if self.source_colorspace == self.target_colorspace:
            self.result = input

        else:
            self.result = cv2.cvtColor(input, self.transforms[self.source_colorspace +
                                                            "2" + self.target_colorspace])
        self.draw()
        return self.result

class Inrange(Module):
    def __init__(self, params):
        if "show" in params.keys():
            self.show = params["show"]
        else:
            self.show = False
        
        super().__init__(params, "Inrange", self.show)


        self.set_ths(params["low_th"], params["high_th"])

    def apply(self, input):
        # _, input_ = list(input.items())[0]


        self.result = cv2.inRange(input, tuple(
            self.low_th), tuple(self.high_th))
        
        self.draw()
        return self.result

    # def draw(self, img):
    #     return cv2.cvtColor(self.result, cv2.COLOR_GRAY2BGR)

    def set_ths(self, low_th_, high_th_):
        self.low_th = list(low_th_)
        self.high_th = list(high_th_)


class Filter_connected_components (Module):
    def __init__(self, params):
        if "show" in params.keys():
            self.show = params["show"]
        else:
            self.show = False
        super().__init__(params, "Filter_connected_components", self.show)

        self.area_low = params["area_low"]
        self.area_high = params["area_high"]
        self.hei_low = params["hei_low"]
        self.hei_high = params["hei_high"]
        self.wid_low = params["wid_low"]
        self.wid_high = params["wid_high"]
        self.den_low = params["den_low"]
        self.den_high = params["den_high"]

    def apply(self, img):  # , area_low = -1, area_high = -1, hei_low = -1, hei_high = -1,
        # wid_low = -1, wid_high = -1, den_low = -1, den_high = -1):
        self.result = filter_connected_components(img, self.area_low,
                                                            self.area_high, self.hei_low, self.hei_high, self.wid_low,
                                                            self.wid_high, self.den_low, self.den_high)
        self.draw()
        return self.result

    # def apply(self, img):
    #     result = np.array(mask)
    #     output = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)

    #     labels_count = output[0]
    #     labels = output[1]
    #     stats = output[2]
    #     sz = stats.shape[0]

    #     def _in_range(value, low, high):
    #         if ((value < low and low != -1) or
    #                 (value > high and high != -1)):
    #             return False

    #         return True

    #     for label_num in range(0, sz):
    #         area = stats[label_num, cv2.CC_STAT_AREA]
    #         height = stats[label_num, cv2.CC_STAT_HEIGHT]
    #         width = stats[label_num, cv2.CC_STAT_WIDTH]
    #         density = float(area) / (height * width)

    #         if (_in_range(area,    self.area_low, self.area_high) == False or
    #             _in_range(height,  self.hei_low,  self.hei_high) == False or
    #             _in_range(width,   self.wid_low,  self.wid_high) == False or
    #                 _in_range(density, self.den_low,  self.den_high) == False):
    #             result[labels == label_num] = 0

    #     return result


class Max_area_cc_bbox(Module):
    def __init__(self, params):
        if "show" in params.keys():
            self.show = params["show"]
        else:
            self.show = False
        super().__init__(params, "Max_area_cc_bbox", self.show)
       


    def apply(self, input):
        # _, input_ = list(input.items())[0]

        self.result = find_max_bounding_box(input)
        if self.show:
            self.draw(input)
        return self.result

    def draw(self, input):
        res = input.copy()
        if self.result[1]:
            bb = self.result[0]
            res = cv2.rectangle(
                res, bb[0], bb[1], (100, 200, 10), 5)

        cv2.imshow(self.type_name, res)
        cv2.waitKey(10)
        # return res

