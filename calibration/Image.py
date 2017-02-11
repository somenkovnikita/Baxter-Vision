import cv2 as cv
import numpy as np


class Video_Camera:

    def __init__(self, width=None, height=None):
        self.width = width
        self.height = height

    def get_frame(self, video_stream):
        ret, frame = video_stream.read()
        if ret:
            if self.width and self.height:
                resized_frame = cv.resize(frame, (self.width, self.height))
                return resized_frame
            else:
                return frame
        else:
            return np.zeros(800, 600)

    def save_frame(self, frame, file_name):
        if cv.waitKey(1) & 0xFF == ord('p'):
            cv.imwrite(file_name, frame)

    def undistort_img(self, img, mapx, mapy):   # Look remap_params func in Calibration class
        undist_img = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
        return undist_img
