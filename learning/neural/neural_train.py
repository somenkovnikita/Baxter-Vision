# -*- coding: utf-8 -*-
import argparse

import cv2
import numpy as np

w = 50
h = 50
siz = w * h

test_hardcode_path = [
    ('assets/letters/training_set/а/pos1.png', np.array([1, 0, 0, 0, 0])),
    ('assets/letters/training_set/а/pos2.png', np.array([1, 0, 0, 0, 0])),
    ('assets/letters/training_set/б/pos3.png', np.array([0, 1, 0, 0, 0])),
    ('assets/letters/training_set/б/pos4.png', np.array([0, 1, 0, 0, 0])),
    ('assets/letters/training_set/в/pos5.png', np.array([0, 0, 1, 0, 0])),
    ('assets/letters/training_set/г/pos6.png', np.array([0, 0, 0, 1, 0])),
    ('assets/letters/training_set/г/pos7.png', np.array([0, 0, 0, 1, 0])),
    ('assets/letters/training_set/д/pos8.png', np.array([0, 0, 0, 0, 1])),
    # ('assets/letters/training_set/а/pos2.png', np.array([0, 1])),
    # ('assets/letters/training_set/б/pos4.png', np.array([1, 0])),
]

def network_init(layers):
    network = cv2.ANN_MLP()
    network.create(np.array(layers))
    return network


def train(images, classes):
    global network
    
    network.train(images, classes, None)

    im1 = cv2.imread('assets/letters/training_set/а/pos2.png')
    # im2 = cv2.imread('assets/letters/test_set/б.jpg')
    im2 = cv2.imread('assets/letters/training_set/й/pos15.png')

    im1 = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    im2 = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)

    im1 = cv2.resize(im1, (h, w)).flatten() / 255.0
    im2 = cv2.resize(im2, (h, w)).flatten() / 255.0

    inputs = np.array([im1, im2])
    ret, out = network.predict(inputs)

    print out


def resize_image(images, h, w):
    s = (h, w)
    result = list()
    for image in images:
        img = cv2.resize(image, dsize=s)
        result.append(img)
    return result

def normalize_images(images):
    results = list()
    for image in images:
        img = 2 * 1.7159 * (image / 255.0) - 1.7159
        results.append(img)
    return results


def vectorize_images(images):
    def img2vec(img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray.flatten()
    return np.array([img2vec(img) for img in images])


def get_pathlist(filename):
    with open(filename) as images_paths:
        return [p.strip() for p in images_paths]


def load(images_path):
    images, classes = list(), list()
    with open(images_path) as paths:
        for img, cls in map(str.split, paths):
            img = cv2.imread(img)
            images.append(img)
            classes.append(cls)
    return images, classes


class NetworkTrainer:
    wight, height = 50, 50
    input_layer_size = wight * height

    def __init__(self, network, in_layer, out_layer):
        self.out = out_layer
        self.in_ = in_layer
        self.network = network
        self.images = images
        self.classes = classes

    def train(self, images, classes):
        w = NetworkTrainer.wight
        h = NetworkTrainer.height

        images = resize_image(images, h, w)
        images = vectorize_images(images)
        images = normalize_images(images)
        images = np.array(images)

        cls = list()
        for c in classes:
            cc = np.zeros(self.out)
            cc[int(c)] = 1.0
            cls.append(cc)
        classes = np.array(cls)

        self.network.train(images, classes, None)
        print 'end'
        

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train letters network")
    parser.add_argument("-i", "--input_file", help="Input file with paths", required=True)

    args = parser.parse_args()
    images, classes = load(args.input_file)

    l1 = NetworkTrainer.input_layer_size
    l2, l3 = 1000, 30
    layers = l1, l2, l3

    net = network_init(layers)
    network_trainer = NetworkTrainer(net, l1, l3)
    network_trainer.train(images, classes)

    pass




