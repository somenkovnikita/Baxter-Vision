import cv2


# Local camera class D.D.M. 2017(c)

class LocalCamera:
    CAMERA_ID = 0
    WIDTH = 640
    HEIGHT = 480

    capture = None
    frame = None
    result = None
    color = -1

    def __init__(self, cam_id=CAMERA_ID, resolution=(WIDTH, HEIGHT)):
        self.capture = cv2.VideoCapture()
        self.capture.open(cam_id)
        # Set params for camera
        (width, height) = resolution
        self.capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)

        if not self.capture.isOpened():
            print "Error camera init"

    # Read frame from camera
    def read_frame(self):
        # Capture frame from camera
        res, fr = self.capture.read()
        # Fix mirror effect on frame
        cv2.flip(fr, 1, fr)
        self.result = res

        if self.color != -1:
            self.frame = cv2.cvtColor(fr, self.color)
        else:
            self.frame = fr

        return self.result

    # Return current frame from camera
    def get_frame(self):
        return self.frame

    # Set color for frame
    def set_color(self, col):
        self.color = col

    def set_fps(self, fps):
        self.capture.set(cv2.cv.CV_CAP_PROP_FPS, fps)

    # WARNING: Params can be unsupported for some web cams
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
