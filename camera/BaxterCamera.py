import rospy, cv2, numpy
from cv_bridge import (CvBridge, CvBridgeError)

from sensor_msgs.msg import Image
import std_srvs.srv
from baxter_interface import CameraController


# TODO: Write simple test for Baxter camera with params
class BaxterCamera:
    def __init__(self, camera_name, resolution=(960, 600)):
        self.resolution = resolution
        size = (resolution[1], resolution[0], 3)
        self.cv_image = numpy.zeros(size, numpy.uint8)
        self.camera_name = camera_name + "_hand_camera"
        self.camera = CameraController(self.camera_name)
        self.bridge = CvBridge()
        self.publisher = None

    def get_frame(self):
        return self.cv_image

    def start(self):
        self.camera.resolution = self.resolution
        self.camera.exposure = -1
        self.camera.gain = -1
        self.camera.white_balance_red = -1
        self.camera.white_balance_green = -1
        self.camera.white_balance_blue = -1

        cam_pub = "/cameras/{0}/image".format(self.camera_name)
        self.publisher = rospy.Subscriber(cam_pub, Image, self._on_got_image)

    def stop(self):
        if self.publisher:
            self.publisher.stop()

    def _on_got_image(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            pass
        cv2.waitKey(3)

    @staticmethod
    def free():
        reset_srv = rospy.ServiceProxy("cameras/reset", std_srvs.srv.Empty)
        rospy.wait_for_service("cameras/reset", timeout=10)
        reset_srv()
