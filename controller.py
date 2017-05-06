import argparse

from baxter.robot import RobotController


def start_controller(aim_word):
    robot = RobotController()
    robot.make_word(aim_word)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Main script for play cube')

    parser.add_argument('-w', '--word', required=True,
                        help='Aim word')

    arguments = parser.parse_args()

    start_controller(arguments.word)
