"""[summary]

Returns:
    [type]: [description]
"""
"""
    Type: sensor_msgs/msg/Image

Publisher count: 1

Node name: gazebo_plugins
Node namespace: /rs_camera
Topic type: sensor_msgs/msg/Image
Endpoint type: PUBLISHER
GID: 01.0f.91.77.29.82.00.00.01.00.00.00.00.00.29.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0
"""
import numpy as np
import cv2
from common import Module

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class Colorspace_transform(Module, Node):
    transforms = {}

    def __init__(self, params):
        Module.__init__(self, params, "colorspace_transform")
        Node.__init__(self, "colorspace_filter_subscriber")

        #print(params["topic"])

        qos = QoSProfile(depth = 10,
                        durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
                        reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.subscription = self.create_subscription(
            #String,
            Image,
            # "sos",
            #'/rs_camera/rs_d435/image_raw',
            params["topic"],
            self.listener,
            qos)

        #self.subscription

        self.bridge = CvBridge()

        self.result = np.zeros((3,3,3), np.uint8)

        self.source_colorspace = params["from"]
        self.target_colorspace = params["to"]

        self.transforms.update({"RGB2BGR": cv2.COLOR_RGB2BGR})

        self.transforms.update({"RGB2GRAY": cv2.COLOR_RGB2GRAY})
        self.transforms.update({"GRAY2RGB": cv2.COLOR_GRAY2RGB})

        self.transforms.update({"RGB2HSV": cv2.COLOR_RGB2HSV})
        self.transforms.update({"HSV2RGB": cv2.COLOR_HSV2RGB})

        self.transforms.update({"RGB2YCrCb": cv2.COLOR_RGB2YCrCb})
        self.transforms.update({"YCrCb2RGB": cv2.COLOR_YCrCb2RGB})

    def apply(self, input):
        inp_listed = list(input.items())

        if (len(inp_listed) > 0):
            _, input_ = list(input.items())[0]
            _, input__ = list(input_.items())[0]

            self.work(input__)

        else:
            self.result = np.ones((20, 20, 3), np.uint8) * 12
            #self.result = np.indices((20, 20, 3))[0].reshape(-1, 20, 20).T * 11

        return self.result

    def work(self, input):
        if self.source_colorspace == self.target_colorspace:
            self.result = input

        else:
            self.result = cv2.cvtColor(input, self.transforms[self.source_colorspace +
                                                            "2" + self.target_colorspace])

    def listener(self, msg):
        #print ("message received")
        input__ = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        self.work (input__)

        cv2.imshow("colorspace transform", self.result)

        # return self.result

    def draw(self, img):
        return self.result


class Inrange(Module):
    def __init__(self, params):
        super().__init__(params, "inrange")

        self.set_ths(params["low_th"], params["high_th"])

    def apply(self, input):
        #print ("inrange input", input)
        _, input_ = list(input.items())[0]

        self.result = cv2.inRange(input_, tuple(self.low_th), tuple(self.high_th))

        return self.result

    def draw(self, img):
        return cv2.cvtColor(self.result, cv2.COLOR_GRAY2BGR)

    def set_ths(self, low_th_, high_th_):
        self.low_th = list(low_th_)
        self.high_th = list(high_th_)


class Max_area_cc_bbox(Module):
    def __init__(self, params):
        super().__init__(params, "max_area_cc_bbox")

        self.bbox_num = params["bbox_num"]

    def apply(self, input):
        _, input_ = list(input.items())[0]

        self.result, _ = self.find_max_bounding_box(input_, self.bbox_num)

        return self.result

    def draw(self, img):
        res = img.copy()

        for s in self.result:
            res = cv2.rectangle(
                res, s[0], s[1], (100, 200, 10), 5)

        return res

    def find_max_bounding_box(self, mask, bbox_num):
        output = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)
        stats = output[2]

        success = True

        sorted_components = stats[np.argsort(stats[:, cv2.CC_STAT_AREA])]
        sorted_components = sorted_components[: -1]
        sorted_components = sorted_components[- min(
            bbox_num, len(sorted_components)):]

        result = []

        for i in range(len(sorted_components)):
            top = sorted_components[i, cv2.CC_STAT_TOP]
            left = sorted_components[i, cv2.CC_STAT_LEFT]
            width = sorted_components[i, cv2.CC_STAT_WIDTH]
            height = sorted_components[i, cv2.CC_STAT_HEIGHT]

            result.append(((left, top), (left + width, top + height)))

        return result, success