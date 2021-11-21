import numpy as np

class CameraModel:
    def __init__(self):
        # super().__init__(params, "model")
        self.focal_length = 300
        self.center_x = 0
        self.center_y = 0

    def get_view_vector_from_pixel(self, pixel_x, pixel_y):
        """
        Getting view vector from camera to pixel 

        Args:
            pixel_x (int): coords in pixels
            pixel_y (int): coords in pixels

        Returns:
            turple: Normalized view vector from camera to pixel
        """
        delta_pixel_x = pixel_x - self.center_x
        delta_pixel_y = pixel_y - self.center_y
        focal_length = self.focal_length
        length = np.sqrt(delta_pixel_x**2 + delta_pixel_y**2 + focal_length**2)

        return (delta_pixel_x/length, delta_pixel_y/length, focal_length/length)

