import ConfigParser

import cv2

import Calibrator

calibrator = None


def init(config):
    global calibrator

    parser = ConfigParser.ConfigParser()
    parser.read(config)
    fn = parser.get('undist', 'input_params')

    calibrator = Calibrator.Calibrator(fn)


def undist(frame):
    global calibrator

    dst = calibrator.calibrate(frame)

    cv2.imshow('Original', frame)
    cv2.imshow('Undistortion', dst)
