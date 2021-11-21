import numpy as np
from .camera_model import CameraModel

class RobotModel:
    def __init__(self):
        self.robot_height = 0.57
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
        alpha = np.atan(view_vector_camera[1])
        beta = np.atan(view_vector_camera[0])

        # robot_alpha- vertical angle of the ball radius-vector relative to the robot body direction
        robot_alpha = alpha - self.cameraTilt

        # the case when the ball is in the horizon
        # if robot_alp <= 0:
            #raise Exception('ball above the horizon')

        # xb, yb - coordinates of the obj in the system, which is turned by cameraPan relative to the robot system
        x_camera = self.h / np.tan(robot_alp)
        y_camera = np.tan(bet) * np.sqrt(self.h ** 2 + xb ** 2)

        # xb_r, yb_r - coordinates of the ball in the robot system
        x_self = x_camera * np.cos(self.cameraPan) + y_camera * np.sin(self.cameraPan)
        y_self = y_camera * np.cos(self.cameraPan) - x_camera * np.sin(self.cameraPan)

        return (x_self, -y_self)
