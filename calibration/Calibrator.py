import cv2
import numpy as np


class Calibrator:
    def __init__(self, filename):
        undist_options = np.load(filename)

        self.mapx = undist_options['mapx']
        self.mapy = undist_options['mapy']
        self.x = undist_options['x']
        self.y = undist_options['y']
        self.w = undist_options['w']
        self.h = undist_options['h']
        self.dist = undist_options['dist']
        self.newcameramtx = undist_options['newcameramtx']
        self.mtx = undist_options['mtx']


    def calibrate(self, frame):
        args = frame, self.mtx, self.dist, \
               None, self.newcameramtx
        dst = cv2.undistort(*args)

        x, y = self.x, self.y
        h, w = self.h, self.w

        return dst[y: y+h, x: x+w]
