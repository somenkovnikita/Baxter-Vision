import ConfigParser

import cv2

from tools.calibrator import Calibrator

calibrator = None


def init(config):
    global calibrator

    parser = ConfigParser.ConfigParser()
    parser.read(config)
    fn = parser.get("undist", "input_params")

    calibrator = Calibrator(fn)


def undist(frame):
    global calibrator

    dst = calibrator.calibrate(frame)

    cv2.imshow("Original", frame)
    cv2.imshow("Undistortion", dst)
