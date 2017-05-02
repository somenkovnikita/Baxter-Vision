import argparse
import os
from time import strftime, gmtime

import cv2
import numpy as np
from nolearn.dbn import DBN
from sklearn.metrics import classification_report

"""Train multilayer perceptron network for recognition of letters"""
"""
Format line in file with marked images:
<path-to-image> <class>
"""

def generate_data():
    data = [
        # np.array([0.0, 0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 255.0]),
        np.array([0.0, 0.0, 255.0, 0.0]),
        np.array([0.0, 0.0, 255.0, 255.0]),
        np.array([0.0, 255.0, 0.0, 0.0]),
        np.array([0.0, 255.0, 0.0, 255.0]),
        np.array([0.0, 255.0, 255.0, 255.0]),
        np.array([255.0, 0.0, 0.0, 0.0]),
        np.array([255.0, 0.0, 0.0, 255.0]),
        # np.array([255.0, 0.0, 255.0, 255.0]),
        # np.array([255.0, 255.0, 255.0, 255.0]),
        # np.array([  0, 0, 0, 1, 0, 0, 0, 0]),
        # np.array([  0, 0, 0, 1, 0, 0, 0, 1]),
        # np.array([  0, 0, 0, 1, 0, 0, 1, 1]),
        # np.array([  0, 0, 0, 1, 0, 1, 1, 1]),
        # np.array([  0, 0, 0, 1, 1, 1, 1, 1]),
        # np.array([  0, 0, 1, 0, 0, 0, 0, 0]),
        # np.array([  0, 0, 1, 0, 0, 0, 0, 1]),
        # np.array([  0, 0, 1, 0, 0, 0, 1, 1]),
        # np.array([  0, 0, 1, 0, 0, 1, 1, 1]),
        # np.array([  0, 0, 1, 0, 1, 1, 1, 1]),
        # np.array([  0, 0, 1, 1, 1, 1, 1, 1]),
        # np.array([  0, 1, 0, 0, 0, 0, 0, 0]),
        # np.array([  0, 1, 0, 0, 0, 0, 0, 1]),
        # np.array([  0, 1, 0, 0, 0, 0, 1, 1]),
        # np.array([  0, 1, 0, 0, 0, 1, 1, 1]),
        # np.array([  0, 1, 0, 0, 1, 1, 1, 1]),
        # np.array([  0, 1, 0, 1, 1, 1, 1, 1]),
        # np.array([  0, 1, 1, 1, 1, 1, 1, 1]),
        # np.array([  1, 0, 0, 0, 0, 0, 0, 0]),
        # np.array([  1, 0, 0, 0, 0, 0, 0, 1]),
        # np.array([  1, 0, 0, 0, 0, 0, 1, 1]),
        # np.array([  1, 0, 0, 0, 0, 1, 1, 1]),
        # np.array([  1, 0, 0, 0, 1, 1, 1, 1]),
        # np.array([  1, 0, 0, 1, 1, 1, 1, 1]),
        # np.array([  1, 0, 1, 1, 1, 1, 1, 1]),
        # np.array([  1, 1, 1, 1, 1, 1, 1, 1]),
    ]

    # for img in data:
        # f = np.array(img.reshape((2, 2)))
        # f2 = cv2.resize(f, (25, 25))
        # cv2.imshow('a', f2)
        # cv2.waitKey()
    data2 = list()
    for img in data:
        data2.append(img / 255.0)
    return np.array(data2), np.array(map(float, range(11)))


def prepare_images(paths):
    images = list()
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        image = cv2.equalizeHist(image)

        image = cv2.resize(image, (25, 25))
        image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 29, 0)

        # cv2.imshow('image', image)
        # cv2.waitKey(0)

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


def get_default_config_filename():
    time = strftime('%H_%M', gmtime())
    config_filename = 'net_{0}.wgh'.format(time)
    return config_filename

if __name__ == '__main__' :
    LETTER_COUNT = 30
    DEFAULT_CONFIG = get_default_config_filename()

    parser = argparse.ArgumentParser(
        description='Train multilayer perceptron network for recognition of letters')

    # required
    parser.add_argument('-i', '--input_layer', type=int, required=True, help='Size of input layer')
    parser.add_argument('-l', '--hidden_layers', metavar='L', type=int, nargs='+', required=True,
                        help='Size(s) of hidden layer')
    parser.add_argument('-s', '--train_set', required=True, help='Path to dataset with marked.list file')

    # options
    parser.add_argument('-c', '--output_config',
                        help='Specify own filename output weight file', default=DEFAULT_CONFIG)
    parser.add_argument('-o', '--output_layer', type=int,
                        help='Size of output layer', default=LETTER_COUNT)

    print 'Reading dataset...'
    args = parser.parse_args()
    paths, classes = load_dataset(args.train_set)

    inputs = prepare_images(paths)
    outputs = prepare_classes(classes, args.output_layer)
    weights = np.ones(len(paths))

    print 'Creating network...'


    input, outputs = generate_data()
    layers = [args.input_layer]
    layers += args.hidden_layers
    layers.append(8)


    # a = generate_data()
    network = DBN(layers,
                  learn_rates=0.1,
                  learn_rate_decays=0.9,
                  epochs=100,
               
                  minibatches_per_epoch=8,
                  verbose=1)

    vv= np.array(map(float, range(8)))
    network.fit(input, vv)
    preds = network.predict(input)
    print classification_report(vv, preds)


    #
    # X = inputs
    # y = outputs
    # model = GaussianNB()
    # model.fit(X, y)
    # model = network
    # print(model)
    #
    # test_set = load_dataset('assets/letters/test_set')
    # i2 = prepare_images(test_set[0])
    # i3 = prepare_classes(test_set[1], args.output_layer)
    #
    #
    # # make predictions
    # expected = i3
    # predicted = model.predict(i2)
    # # summarize the fit of the model
    # print classification_report(expected, predicted)

