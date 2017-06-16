import argparse
import os

import cv2
import numpy as np
from sklearn.metrics import classification_report

from vision.letterrecognizers import CaffeNeural


def prepare_images(paths):
    def read_and_resize(p):
        return cv2.resize(cv2.imread(p), (28, 28))
    return map(read_and_resize, paths)


def prepare_classes(classes):
    return np.array(map(int, classes))


def load_dataset(dataset_dir):
    import random
    filename = os.path.join(dataset_dir, 'marked.list')
    with open(filename) as paths:
        lines = paths.readlines()
        random.shuffle(lines)
        fns, cls = zip(*map(str.split, lines))

    def j(x):
        return os.path.join(dataset_dir, x)
    return map(j, fns), cls


def prepare_inputs_outputs(config):
    print 'Reading dataset...'

    paths, classes = load_dataset(config)
    inputs = prepare_images(paths)
    outputs = prepare_classes(classes)

    return inputs, outputs


if __name__ == '__main__' :
    DEFAULT_CONFIG = os.path.join('config', 'letter_recognizer.svm')

    parser = argparse.ArgumentParser(description='Train svm model')

    parser.add_argument('-m', '--model', required=True,
                        help='Model file for Caffe network')
    parser.add_argument('-w', '--weight', required=True,
                        help='Weights for for Caffe model')
    parser.add_argument('-t', '--test_set', required=True,
                        help='Directory with test set with market.list')

    print 'Reading dataset...'
    args = parser.parse_args()

    model = CaffeNeural(args.model, args.weight)

    images, outputs = prepare_inputs_outputs(args.test_set)
    predicts = model.letters(images)
    print classification_report(outputs, predicts)
