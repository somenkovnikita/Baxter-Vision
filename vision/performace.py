import argparse

"""Print performance for test set"""


def performance(recognizer):
    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Print performance for test set')
    parser.add_argument('-c', '-config', required=True,
                        help='Neural network weights in special OpenCV format')
    parser.add_argument('-i', '-input',
                        help='Neural network weights in special OpenCV format')

    args = parser.parse_args()
