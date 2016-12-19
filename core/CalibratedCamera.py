import cv2


class CalibratedCamera:
    def __init__(self, camera, config):
        self.camera = camera
        self.config = config

    def get_image(self):
        image = camera.get_image()
        # TODO: calc undist image
        return image
