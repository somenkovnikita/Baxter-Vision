# -*- coding: utf8 -*-

import ConfigParser
import argparse
import os
import random
import signal

import rospy

from baxter.camera import Camera
from baxter.hand import HandMover
from tools.maps import ClassMap
from vision.cubedetectors import CascadeCubeDetector
from vision.interface import ICubeDetector
from vision.interface import ILetterRecognizer
from vision.letterrecognizers import SVMLetterRecognizer


class RobotController:
    """
    Main controller for manipulation with cubes
    """
    def __init__(self, ini_file):
        """
        Construct from ini file with some params.
        You can get new config with robot_calibrate.py
        
        :param ini_file: robot file config 
        """
        parser = ConfigParser.ConfigParser()
        parser.read(ini_file)

        print 'loading config...', parser.sections()

        cube_detector = parser.get('vision', 'cube_detector')
        cube_detector_config = parser.get('vision', 'cube_detector_config')
        cube_detector_class = RobotController._load_component(cube_detector)
        self.cube_detector = cube_detector_class(cube_detector_config)

        letter_detector = parser.get('vision', 'letter_detector')
        letter_detector_config = parser.get('vision', 'letter_detector_config')
        letter_detector_class = RobotController._load_component(letter_detector)
        self.letter_recognizer = letter_detector_class(letter_detector_config)

        class_map = parser.get('vision', 'class_map')
        self.alphabet = ClassMap(class_map)

        start_position = parser.get('positions', 'start_position')
        self.start_pose = map(float, start_position.split(';'))
        self.height_plane = float(parser.get('positions', 'height_plane'))

        end_position = parser.get('positions', 'end_position')
        self.end_position = map(float, end_position.split(';'))

        hand = parser.get('positions', 'hand')
        self.hand = HandMover(hand)
        self.camera = Camera(hand + '_hand')

    def make_word(self, word):
        """
        Main method, than make word from start position
        
        :param word: aim word
        :param start_point:  start position for start first letter
        :return: True if word suc
        """
        rc = RobotController

        # TODO: shift_aim must be load from cfg
        shift_aim = 0.01

        to_aim = list(self.end_position)
        self.set_start_pose()
        for letter in word:
            roi_rect = self.next_aim(letter)
            from_aim = rc.get_rect_center(roi_rect)
            self.move_cube(from_aim, to_aim)
            to_aim[0] += shift_aim
            self.set_start_pose()

    def set_start_pose(self):
        self.hand.try_move(*self.start_pose)

    def random_move(self):
        while True:
            dx = 0.5 * random.random() - 0.25
            dy = 0.5 * random.random() - 0.25
            if self.hand.try_move(dx, dy):
                return

    def find_cubes(self):
        for attempt in range(20):
            print 'Try find cubes attempt ', attempt

            frame = self.camera.get_frame()
            cubes = self.cube_detector.cubes(frame)

            def cutter(rect):
                x, y, h, w = rect
                return frame[y: y + h, x: x + w]

            if len(cubes) != 0:
                print 'Found %d cubes' % len(cubes)
                return map(cutter, cubes), cubes

        raise RuntimeError('cubes not find')

    def find_letter(self, cubes_image, aim_letter):
        print 'Try find letter', aim_letter
        letter_class = self.alphabet.get_class(aim_letter)
        letters = self.letter_recognizer.letters(cubes_image)
        if letter_class in letters:
            print 'Found', aim_letter, 'letter'
            return letters.index(letter_class)
        raise RuntimeError('not find letter' + str(aim_letter))

    def next_aim(self, letter):
        for attempt in range(30):
            cubes, rects = self.find_cubes()
            index = self.find_letter(cubes, letter)
            if index:
                return rects[index]

    def aim_to(self, aim):
        """
        The robot's arm becomes above the target
        
        :param aim: coordinates aim 
        :return: True if move success False otherwise
        """
        # TODO: must be load from cfg

        print 'aim to', aim
        aim_w_percent = 0.56
        aim_h_percent = 0.45
        robot_to_px = 0.157

        w, h = self.camera.resolution
        aim_x = int(round(aim_w_percent * w))
        aim_y = int(round(aim_h_percent * h))

        pose = self.hand.get_current_pose()
        # TODO: must be load from cfg
        z_px = -330.0 * pose[2] + 201.0

        dx = robot_to_px * (aim[0] - aim_y) / z_px
        dy = robot_to_px * (aim[1] - aim_x) / z_px

        return self.hand.try_delta_move(dx=dx, dy=dy)

    def take(self):
        pose = self.hand.get_current_pose()
        print self.hand.set_gripper(True)
        print self.hand.try_move(z=self.height_plane)
        print self.hand.set_gripper(False)
        print self.hand.try_move(z=pose[2])

    def give_back(self):
        pose = self.hand.get_current_pose()
        print self.hand.try_move(z=self.height_plane)
        print self.hand.set_gripper(True)
        print self.hand.try_move(z=pose[2])

    def move_cube(self, from_aim, to_aim):
        print 'moving', from_aim, '->', to_aim
        self.aim_to(from_aim)
        self.take()
        self.aim_to(to_aim)
        self.give_back()

    def set_letter_detector(self, letter_recognizer):
        # type: (ILetterRecognizer) -> None
        self.letter_recognizer = letter_recognizer

    def set_cube_detector(self, cube_detector):
        # type: (ICubeDetector) -> None
        self.cube_detector = cube_detector

    @staticmethod
    def get_rect_center(rect):
        cx = rect[0] + rect[2] / 2
        cy = rect[1] + rect[3] / 2
        return cx, cy



    @staticmethod
    def _load_component(detector):
        components = detector.split('.')
        mod = __import__(components[0])
        for comp in components[1:]:
            mod = getattr(mod, comp)
        return mod


def start_controller(aim_word):
    robot = RobotController('left')

    config = 'config/cube_detector_10k.xml'
    detector = CascadeCubeDetector(config)
    robot.set_cube_detector(detector)

    config = 'config/letter_recognizer.svm'
    detector = SVMLetterRecognizer(config)
    robot.set_letter_detector(detector)

    robot.make_word(aim_word)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Main script for play cube')

    parser.add_argument('-w', '--word', required=True,
                        help='Aim word')
    parser.add_argument('-c', '--config', required=True,
                        help='Config with info about detectors, start positions e.t')

    arguments = parser.parse_args()
    rospy.init_node('cube_puzzle2')

    import cv2

    import sys

    reload(sys)
    sys.setdefaultencoding('utf-8')


    robot = RobotController(arguments.config)

    from tools.image_cutter import ClickChecker
    cv2.namedWindow('df')
    clicker = ClickChecker('df')

    while True:
        frame = robot.camera.get_frame()
        cv2.imshow('df', frame)
        key = cv2.waitKey(1) & 0xff
        if key == 27:
            break
        if key == ord('w'):
            robot.take()
        if key == ord('t'):
            robot.give_back()
        if key == ord('l'):
            robot.make_word(arguments.word.decode())
        if key == ord('r'):
            robot.random_move()
        c = clicker.get_clicks()
        for cc in c:
            robot.aim_to(cc[::-1])

    os.kill(os.getpid(), signal.SIGINT)

    rospy.spin()

    # robot.take()
