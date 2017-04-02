import cv2
import numpy as np


class Calibrator:
    def __init__(self, filename):
        dist_options = np.load(filename)

        self.dist = dist_options["dist"]
        self.new_camera_mtx = dist_options["new_camera_mtx"]
        self.mtx = dist_options["mtx"]

    def calibrate(self, frame):
        args = frame, self.mtx, self.dist, \
               None, self.new_camera_mtx
        dst = cv2.undistort(*args)

        # x, y = self.x, self.y
        # h, w = self.h, self.w

        return dst
