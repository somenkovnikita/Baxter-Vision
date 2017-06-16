import threading

import numpy
import rospy
import std_srvs.srv
from baxter_interface import CameraController
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image


class Camera:
    """
    Baxter camera 
    Help: http://sdk.rethinkrobotics.com/wiki/Cameras
    """

    default_width = 640
    default_height = 400
    default_resolution = default_width, default_height

    def __init__(self, camera_name, resolution=(default_width, default_height)):
        # type: (str, tuple) -> None
        """
        Create camera instance by camera name
        
        :param camera_name: 'left_hand' or 'right_hand' or 'head' 
        :param resolution: output image resolution
        """
        self.resolution = resolution

        shape = resolution[1], resolution[0], 3
        self.cv_image = numpy.zeros(shape, numpy.uint8)
        self.lock = threading.Lock()

        self.camera_name = camera_name + "_camera"
        self.camera = CameraController(self.camera_name)
        self.camera.resolution = self.resolution

        cam_pub = "/cameras/{0}/image".format(self.camera_name)
        self.publisher = rospy.Subscriber(cam_pub, Image, self._on_update_image)
        self.calibrator = None

        self.bridge = CvBridge()

    def get_frame(self):
        # type: (Camera) -> numpy.array
        """
        Get next frame from camera flow
        
        :return: OpenCV image 
        """
        self.lock.acquire()
        image = self.cv_image.copy()
        self.lock.release()
        if self.calibrator:
            return self.calibrator.calibrate(image)
        return image

    def free(self):
        # type: (Camera) -> None
        """
        Free camera
        
        :return: None 
        """
        if self.publisher:
            self.publisher.stop()
        reset_srv = rospy.ServiceProxy("cameras/reset", std_srvs.srv.Empty)
        rospy.wait_for_service("cameras/reset", timeout=10)
        reset_srv()

    def set_calibrator(self, calibrator):
        self.calibrator = calibrator

    def set_gain(self, gain):
        self.camera.gain = gain

    def set_exposure(self, exposure):
        self.camera.exposure = exposure

    def set_fps(self, fps):
        self.camera.fps = fps

    def set_window(self, size=(default_width, default_height)):
        self.camera.window = size

    def set_balance(self, r, g, b):
        self.camera.white_balance_red = r
        self.camera.white_balance_green = g
        self.camera.white_balance_blue = b

    def _on_update_image(self, data):
        self.lock.acquire()
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print "Camera: Error camera frame " + e
        self.lock.release()

    def __del__(self):
        self.free()
