import argparse

"""Print performance for test set"""

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Print performance neural network for test set')
    parser.add_argument('-c', '-config', help='Neural network weights in special OpenCV format', required=True)
    parser.add_argument('-c', '-input', help='Neural network weights in special OpenCV format')

    args = parser.parse_args()





