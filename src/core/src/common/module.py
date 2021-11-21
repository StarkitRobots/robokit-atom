import numpy as np
import json
import cv2

#from .input_output import Source as Source_
#from .processor import Processors as Processor_
import math


class Module:
    def __init__(self, params, type_name="module", show = False):
        self.params = params
        self.type_name = type_name
        self.show = show
        self.result = None

    def apply(self, input):
        pass

    def draw(self):
        if self.show:
            cv2.imshow(self.type_name, self.result)
            cv2.waitKey(10)


class Source(Module):
    def __init__(self, params, type_name):
        Module.__init__(self, type_name)

    def apply(self, input):
        print("Source.apply() called")


class Camera(Source):
    def __init__(self, params):
        Source.__init__(self, params, "camera")
        # print ("aa fyk", type(params ["data_path"]) = type)
        self.source = Source_(params["data_path"])

    def apply(self, input):
        self.frame = self.source.get_frame()
        return self.frame

    def draw(self, img):
        return self.frame


class Keyboard(Source):
    def __init__(self, params):
        Module.__init__(self, params, "keyboard")

    def apply(self, input):
        return cv2.waitKey(50) & 0xFF
