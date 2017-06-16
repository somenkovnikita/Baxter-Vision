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
        :return: True if word suc
        """
        rc = RobotController

        # TODO: shift_aim must be load from cfg
        shift_aim = 0.06

        to_aim = list(self.end_position)
        self.set_start_pose()
        for letter in word:
            print 'roi_rect'
            roi_rect = self.next_aim(letter)
            if roi_rect is None:
                print 'Not found next letter'
                break
            print 'get_rect_center'
            from_aim = rc.get_rect_center(roi_rect)
            print 'move_cube'
            self.move_cube(from_aim, to_aim)
            to_aim[1] += shift_aim
            print 'set_start_pose'
            self.set_start_pose()

    def set_start_pose(self):
        print 'Moving to start pose, result: ',
        print self.hand.try_move(*self.start_pose)
        print self.hand.rotate_gripper(0.0)

    def random_move(self):
        for attempt in range(3):
            step = 0.125
            dx = step * random.random() - step / 2
            dy = step * random.random() - step / 2
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
        recognized_letter_classes = self.letter_recognizer.letters(cubes_image)

        print 'Found', u' '.join(map(self.alphabet.get_letter, recognized_letter_classes))

        if letter_class in recognized_letter_classes:
            print 'Aim letter found success!'
            return recognized_letter_classes.index(letter_class)

    def next_aim(self, letter):
        for attempt in range(2):
            cubes, rects = self.find_cubes()
            index = self.find_letter(cubes, letter)
            letter_found = index is not None
            if letter_found:
                return rects[index]
            self.random_move()

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
        to_do = [
            lambda: self.hand.set_gripper(True),
            lambda: self.hand.try_move(z=self.height_plane),
            lambda: self.hand.set_gripper(False),
            lambda: self.hand.try_move(z=pose[2])
        ]
        print all([action() for action in to_do])

    def give_back(self):
        pose = self.hand.get_current_pose()
        print self.hand.try_move(z=self.height_plane)
        print self.hand.set_gripper(True)
        print self.hand.try_move(z=pose[2])
        print self.hand.rotate_gripper(0.0)

    def move_cube(self, from_aim, to_aim):
        print 'moving', from_aim, '->', to_aim
        self.aim_to(from_aim)
        self.take()
        self.hand.try_move(x=to_aim[0], y=to_aim[1])
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
        print rect, cx, cy
        return cy, cx

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


def run_look_at_mode(robot):
    import cv2
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Main script for play cube')

    parser.add_argument('-w', '--word', required=True,
                        help='Aim word')
    parser.add_argument('-c', '--config', required=True,
                        help='Config with info about detectors, start positions e.t')

    arguments = parser.parse_args()
    rospy.init_node('cube_puzzle')

    robot = RobotController(arguments.config)
    robot.make_word(arguments.word)