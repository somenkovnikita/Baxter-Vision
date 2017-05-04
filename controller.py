import argparse

from baxter.camera import Camera
from baxter.hands import HandMover
from tools.maps import ClassMap
from vision.interface import ICubeDetector
from vision.interface import ILetterRecognizer


class RobotController:
    def __init__(self):
        self.cube_detector = ICubeDetector()
        self.letter_recognizer = ILetterRecognizer()
        self.alphabet = ClassMap('config/class_letter.txt')
        self.hand = HandMover('right')
        self.camera = Camera('right_hand')

    def make_word(self, word, start_point=(0, 0)):
        for letter in word:
            cubes, rects = self.find_cubes()
            letters = self.find_letter(cubes, cubes)
            self.move_cube()

    def set_start_pose(self):
        pass

    def random_move(self):
        pass

    def find_cubes(self):
        for attempt in range(20):
            print 'Try find cubes attempt ', attempt

            frame = self.camera.get_frame()
            cubes = self.cube_detector.cubes(frame)

            if cubes:
                print 'Found %d cubes' % len(cubes)
                cutter = lambda r: frame[r[1]: r[3], r[0]: r[2]]
                return map(cutter, cubes), cubes

            self.random_move()
        raise Exception('Cubes not find!')

    def find_letter(self, cubes_image, aim_letter):
        print 'Try find letter', aim_letter
        letter_class = self.alphabet.get_class(aim_letter)
        letters = self.letter_recognizer.letters(cubes_image)
        if letter_class in letters:
            print 'Found', aim_letter, 'letter'
            return letters.index(letter_class)

    def move_cube(self):
        pass

    def set_letter_detector(self, letter_recognizer):
        # type: (RobotController, ILetterRecognizer) -> None
        self.letter_recognizer = letter_recognizer

    def set_cube_detector(self, cube_detector):
        # type: (RobotController, ICubeDetector) -> None
        self.cube_detector = cube_detector


def start_controller(aim_word):
    raise Exception('Not working(')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Main script for play cube')

    parser.add_argument('-w', '--word', required=True,
                        help='Aim word')

    arguments = parser.parse_args()

    start_controller(arguments.word)
