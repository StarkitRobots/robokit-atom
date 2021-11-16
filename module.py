import numpy as np
import json
import cv2

from .input_output import Source as Source_
from .processor import Processors as Processor_
import math


class Module:
    def __init__(self, params, type_name="module"):
        self.params = params
        self.type_name = type_name

    def apply(self, input):
        pass

    def draw(self, img):
        return img


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
