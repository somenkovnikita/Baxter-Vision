import argparse
import os

import cv2
import numpy as np
from sklearn.externals import joblib
from sklearn.metrics import classification_report
from sklearn.naive_bayes import GaussianNB


def prepare_images(paths):
    images = list()
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        image = cv2.equalizeHist(image)

        image = cv2.resize(image, (25, 25))
        # image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 29, 0)
        # cv2.imshow('1,', image)
        # cv2.waitKey()
        images.append(image.flatten().astype(float) / 255.0)
    return np.array(images)


def prepare_classes(classes, osize):
    result = list()
    for i in map(int, classes):
        ans = np.zeros(osize)
        ans[i] = 1.0
        result.append(ans)
    return np.array(map(int, classes))


def load_dataset(dataset_dir):
    filename = os.path.join(dataset_dir, 'marked.list')
    with open(filename) as paths:
        fns, cls = zip(*map(str.split, paths))

    def j(x):
        return os.path.join(dataset_dir, x)
    return map(j, fns), cls


if __name__ == '__main__' :
    DEFAULT_CONFIG = os.path.join('config', 'letter.bayesmodel')

    parser = argparse.ArgumentParser(description='Train naive bayes model')

    parser.add_argument('-i', '--input_dir', required=True,
                        help='Directory with dataset with market.list')
    parser.add_argument('-o', '--output_dir', default=DEFAULT_CONFIG,
                        help='Specify file path to save model')

    print 'Reading dataset...'
    args = parser.parse_args()
    paths, classes = load_dataset(args.input_dir)

    inputs = prepare_images(paths)
    outputs = prepare_classes(classes, 30)

    model = GaussianNB()
    model.fit(inputs, outputs)

    preds = model.predict(inputs)
    print classification_report(outputs, preds)

    joblib.dump(model, 'p.out')
