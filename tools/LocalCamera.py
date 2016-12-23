import numpy as np
import cv2


class LocalCamera:
    def __init__(self, resolution, camera_index=0):
        self.camera = cv2.VideoCapture(camera_index)
        self.width, self.height = resolution
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, self.width)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, self.height)
        self.empty = np.zeros((self.width, self.height, 3))

    def __del__(self):
        if self.camera:
            self.camera.release()

    def get_image(self):
        ret, frame = self.camera.read()
        return frame if ret is not None else self.empty
