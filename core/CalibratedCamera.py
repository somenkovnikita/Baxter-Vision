class CalibratedCamera:
    def __init__(self, camera, config):
        self.camera = camera
        self.config = config

    def get_image(self):
        image = self.camera.get_image()
        # TODO: calc undist image
        return image
