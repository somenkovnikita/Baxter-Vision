import cv2
import numpy

__author__ = 'ddm'


class LocalCamera:

    default_camera_id = 0
    default_width = 640
    default_height = 480
    default_resolution = default_width, default_height

    def __init__(self, camera_id=default_camera_id,
                 resolution=default_resolution):
        # type: (LocalCamera, int, tuple) -> None
        """
        Initialize camera by index and camera resolution
        
        :param camera_id: camera index, by default - default system camera
        :param resolution: resolution, by default 640x480
        """
        self.capture = cv2.VideoCapture()
        self.capture.open(camera_id)
        self.resolution = resolution

        width, height = resolution
        self.capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)

        if not self.capture.isOpened():
            raise IOError("LocalCamera: Error camera init")

    def get_frame(self):
        # type: (LocalCamera) -> numpy.array
        """
        Return next frame from camera flow
        
        :return: OpenCV image 
        """
        result, frame = self.capture.read()

        if not result:
            return numpy.zeros(shape=self.resolution)

        return frame

    # WARNING: Params can be unsupported for some web cams
    def set_fps(self, fps):
        self.capture.set(cv2.cv.CV_CAP_PROP_FPS, fps)

    def set_brightness(self, brightness):
        self.capture.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, brightness)

    def set_contrast(self, contrast):
        self.capture.set(cv2.cv.CV_CAP_PROP_CONTRAST, contrast)

    def set_hue(self, hue):
        self.capture.set(cv2.cv.CV_CAP_PROP_HUE, hue)

    def set_exposure(self, exposure):
        self.capture.set(cv2.cv.CV_CAP_PROP_EXPOSURE, exposure)

    def set_gain(self, gain):
        self.capture.set(cv2.cv.CV_CAP_PROP_GAIN, gain)

    def set_saturation(self, saturation):
        self.capture.set(cv2.cv.CV_CAP_PROP_SATURATION, saturation)

    def free(self):
        self.capture.release()

    def __del__(self):
        self.free()
