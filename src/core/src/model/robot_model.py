import numpy as np
from .camera_model import CameraModel

class RobotModel:
    def __init__(self):
        self.robot_height = 0.24
        self.camera_pan = 0
        self.camera_tilt = 0
        self.camera_model = CameraModel()

    def update_camera_pan_tilt(self, camera_pan, camera_tilt):
        self.camera_pan = camera_pan
        self.camera_tilt = camera_tilt

    def image2self(self, pixel_x, pixel_y):
        # view_vector_camera radius-vector relative to the camera 0, 0 vector direction
        view_vector_camera = self.camera_model.get_view_vector_from_pixel(pixel_x, pixel_y)

        # alpha, beta - vertical and horizontal angles of the ball radius-vector relative to the camera 0,0 direction
        alpha = np.arctan(view_vector_camera[1])
        beta = np.arctan(view_vector_camera[0])

        # robot_alpha- vertical angle of the ball radius-vector relative to the robot body direction
        robot_alpha = alpha - self.camera_tilt

        # the case when the ball is in the horizon
        # if robot_alp <= 0:
            #raise Exception('ball above the horizon')

        # xb, yb - coordinates of the obj in the system, which is turned by cameraPan relative to the robot system
        x_camera = self.robot_height / np.tan(robot_alpha)
        y_camera = np.tan(beta) * np.sqrt(self.robot_height ** 2 + x_camera ** 2)

        # xb_r, yb_r - coordinates of the ball in the robot system
        x_self = x_camera * np.cos(self.camera_pan) + y_camera * np.sin(self.camera_pan)
        y_self = y_camera * np.cos(self.camera_pan) - x_camera * np.sin(self.camera_pan)

        return (x_self, -y_self)
