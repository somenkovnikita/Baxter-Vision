import argparse
import timeit

import cv2

from vision.cubedetectors import CascadeCubeDetector

"""This script for calculation quality of cascade classifier"""


def performance(images, detector):
    color = 255, 255, 0
    for image_ in images:
        image = image_.copy()
        image = cv2.resize(image, None, fx=0.5, fy=0.5)
        start = timeit.default_timer()
        cubes = detector.cubes(image)
        stop = timeit.default_timer()
        print stop - start

        # for x, y, w, h in cubes:
        #     start = x, y
        #     end = x + w, y + h
        #     cv2.rectangle(image, start, end, color, 2)
        # cv2.imshow('Look and count!', image)
        # cv2.waitKey()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='This script for calculation quality of cascade classifier')

    parser.add_argument('-i', '--input', required=True,
                        help='File with paths to images')
    parser.add_argument('-c', '--config', required=True,
                        help='Configuration file with trained cascade')

    args = parser.parse_args()

    with open(args.input) as images_path:
        paths = map(str.strip, images_path)
        images = map(cv2.imread, paths)

    detector = CascadeCubeDetector(args.config)

    performance(images, detector)
