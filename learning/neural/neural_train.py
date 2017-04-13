# -*- coding: utf-8 -*-
import argparse

import cv2
import numpy as np

network = None


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

def network_init():
    global network
    network = cv2.ANN_MLP()
    network.create(np.array([siz, 30 * siz, 50, 20, 15, 5]))


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


def resize_image(images, h=h, w=w):
    s = (h, w)
    result = list()
    for image in images:
        img = cv2.resize(image, s)
        result.append(img)
    return result

def normalize_images(images):
    results = list()
    for image in images:
        img = image / 255.0
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


def load_images(images_paths):
    images = list()
    for path in images_paths:
        image = cv2.imread(path.strip())
        images.append(image)
    return images

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train letters network")
    parser.add_argument("-i", "--input_dir", help="Input file with paths", required=True)

    args = parser.parse_args()

    # paths = get_pathlist()
    paths = [p for p, c in test_hardcode_path]
    classes = np.array([c for p, c in test_hardcode_path], dtype=np.float32)

    images = load_images(paths)
    images = resize_image(images)
    images = vectorize_images(images)
    images = normalize_images(images)
    images = np.array(images)

    network_init()
    train(images, classes)

    pass




