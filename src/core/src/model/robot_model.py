import numpy as np
from .camera_model import CameraModel

class RobotModel:
    def __init__(self):
        self.base_height = 0.415
        self.neck_height = 0.035
        self.robot_height = self.base_height + self.neck_height
        self.tilt_bias = 0.0

        self.camera_pan = 0
        self.camera_tilt = 0
        self.camera_model = CameraModel()

    def update_camera_pan_tilt(self, camera_pan, camera_tilt):
        self.camera_pan = camera_pan
        self.camera_tilt = camera_tilt
        self.robot_height = self.base_height + self.neck_height * np.cos(self.camera_tilt)

    def tilt_rotate(self, ang, vec):
        rot_matrix = np.array(
            [
                [np.cos(ang), 0.0, -np.sin(ang)],
                [0.0, 1.0, 0.0],
                [np.sin(ang), 0.0, np.cos(ang)]
            ]
        )
        return np.dot(rot_matrix, vec)

    def pan_rotate(self, ang, vec):
        rot_matrix = np.array(
            [
                [np.cos(ang), -np.sin(ang), 0.0],
                [np.sin(ang), np.cos(ang), 0.0],
                [0.0, 0.0, 1.0]
            ]
        )
        return np.dot(rot_matrix, vec)
        

    def image2self(self, pixel_x, pixel_y, height):
        # view_vector_camera radius-vector relative to the camera 0, 0 vector direction
        view_vector_camera = self.camera_model.get_view_vector_from_pixel(pixel_x, pixel_y)

        # alpha, beta - vertical and horizontal angles of the ball radius-vector relative to the camera 0,0 direction
        # alpha = np.arctan(view_vector_camera[1])
        # beta = np.arctan(view_vector_camera[0])

        # robot_alpha- vertical angle of the ball radius-vector relative to the robot body direction
        # robot_alpha = alpha - self.camera_tilt

        # the case when the ball is in the horizon
        # if robot_alp <= 0:
            #raise Exception('ball above the horizon')

        x_homog, y_homog, _ = view_vector_camera
        tmp_vec = self.tilt_rotate(self.camera_tilt, [1.0, x_homog, y_homog])
        tmp_vec = self.pan_rotate(self.camera_pan, tmp_vec)
        tmp_vec = self.tilt_rotate(self.tilt_bias, tmp_vec)

        x_camera = -(self.robot_height - height) / tmp_vec[2]
        y_camera = x_camera * x_homog
        z_camera = x_camera * y_homog

        self_vec = self.tilt_rotate(self.camera_tilt, [x_camera, y_camera, z_camera])
        self_vec = self.pan_rotate(self.camera_pan, self_vec)
        self_vec = self.tilt_rotate(self.tilt_bias, self_vec)

        x_self = self_vec[0] + self.neck_height * np.sin(-self.camera_tilt) * np.cos(self.camera_pan)
        y_self = self.vec[1] + self.neck_height * np.sin(-self.camera_tilt) * np.sin(self.camera_pan)

        return (x_self, y_self)

        # xb, yb - coordinates of the obj in the system, which is turned by cameraPan relative to the robot system
        # x_camera = self.robot_height / np.tan(robot_alpha)
        # y_camera = np.tan(beta) * np.sqrt(self.robot_height ** 2 + x_camera ** 2)

        # # xb_r, yb_r - coordinates of the ball in the robot system
        # x_self = x_camera * np.cos(self.camera_pan) + y_camera * np.sin(self.camera_pan)
        # y_self = y_camera * np.cos(self.camera_pan) - x_camera * np.sin(self.camera_pan)

        # y_self = -1 * y_self

        # x_self += self.neck_height * np.sin(-self.camera_tilt) * np.cos(self.camera_pan)
        # y_self += self.neck_height * np.sin(-self.camera_tilt) * np.sin(self.camera_pan)

        # return (x_self, -y_self)
