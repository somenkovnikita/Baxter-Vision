import rospy
from cv_bridge import (CvBridge, CvBridgeError)
from sensor_msgs.msg import Image
from baxter_interface import CameraController
import numpy as np


class camera:
    def __init__(self, camera_type, size):
        width, height = size
        self.name = camera_type
        self.cam = CameraController(camera_type)
        self.cam.resolution = size
        self.cv_image = np.zeros((width, height, 3), np.uint8)

    def setup_camera(self):
        self.cam.exposure = -1
        self.cam.gain = -1
        self.cam.white_balance_red = -1
        self.cam.white_balance_green = -1
        self.cam.white_balance_blue = -1

    def callback(self, image):
        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(image, "bgr8")

    def get_image(self):
        return self.cv_image

    def start(self):
        cam_pub = '/cameras/{0}/image'.format(self.name)
        print cam_pub
        rospy.Subscriber(cam_pub, Image, self.callback)

    def stop(self):
        pass
