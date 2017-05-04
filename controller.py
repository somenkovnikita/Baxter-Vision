import argparse


class RobotController:
    def __init__(self):
        self.cube_detector = None
        self.letter_detector = None

    def run(self):
        pass

    
def start_controller(aim_word):
    raise Exception('Not working(')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Main script for play cube')

    parser.add_argument('-w', '--word', required=True,
                        help='Aim word')

    arguments = parser.parse_args()

    start_controller(arguments.word)
