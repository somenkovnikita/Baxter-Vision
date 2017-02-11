import rospy, cv2, numpy
from cv_bridge import (CvBridge, CvBridgeError)

from sensor_msgs.msg import Image
import std_srvs.srv
from baxter_interface import CameraController


class BaxterCamera:
    WIDTH = 1280
    HEIGHT = 800

    has_frame = False

    def __init__(self, camera_name, resolution=(WIDTH, HEIGHT)):
        self.resolution = resolution
        size = (resolution[1], resolution[0], 3)
        self.cv_image = numpy.zeros(size, numpy.uint8)
        self.camera_name = camera_name + "_camera"
        self.camera = CameraController(self.camera_name)
        self.camera.resolution = self.resolution
        self.bridge = CvBridge()
        self.publisher = None
        cam_pub = "/cameras/{0}/image".format(self.camera_name)
        self.publisher = rospy.Subscriber(cam_pub, Image, self.on_update_image)

    def read_frame(self):
        return self.has_frame

    def get_frame(self):
        return self.cv_image

    # CAMERA PARAMS
    def set_gain(self, gain):
        self.camera.gain = gain

    def set_exposure(self, exposure):
        self.camera.exposure = exposure

    def set_fps(self, fps):
        self.camera.fps = fps

    def set_balance(self, r, g, b):
        self.camera.white_balance_red = r
        self.camera.white_balance_green = g
        self.camera.white_balance_blue = b

    def on_update_image(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.has_frame = True
        except CvBridgeError as e:
            print "BaxterCamera: Error camera frame " + e
            self.has_frame = False
            pass
        cv2.waitKey(1)

    def free(self):
        if self.publisher:
            self.publisher.stop()
        reset_srv = rospy.ServiceProxy("cameras/reset", std_srvs.srv.Empty)
        rospy.wait_for_service("cameras/reset", timeout=10)
        reset_srv()
