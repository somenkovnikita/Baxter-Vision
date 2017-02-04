import json
import sys
import threading
import time
from argparse import ArgumentParser

import cv2

from vision import (NeuralNetwork,
                    NeuralNetworkParameters)

trained = False


def waiting_draw(prompt, frequency=3.0):
    chars = ['-', '\\', '|', '/']
    index = 0
    while not trained:
        index = (index + 1) % len(chars)
        print_string = '\r' + prompt + chars[index]
        sys.stdout.write(print_string)
        sys.stdout.flush()
        time.sleep(1.0 / frequency)
    print ''


def train(layers, images, classes, parameters):
    # type: (tuple, list, list, dict) -> NeuralNetwork
    global trained
    param = NeuralNetworkParameters()
    if parameters is not None:
        param.set_parameters(**parameters)

    thread = threading.Thread(target=waiting_draw,
                              args=('Training network ',))
    thread.daemon = True
    thread.start()

    network = NeuralNetwork(layers)
    iteration = network.train(images, classes, param)
    trained = True
    while thread.isAlive():
        pass
    print 'Training done with %d iteration' % iteration
    return network


def load_images(size, filename):
    images = list()
    classes = list()
    with open(filename) as file_path:
        for line in file_path:
            splitted = line.strip().split(',')
            image_fn, _class = splitted
            image = cv2.imread(image_fn, cv2.IMREAD_GRAYSCALE)
            image = cv2.resize(image, size)
            image = cv2.normalize(image, alpha=-1.0, beta=1.0,
                                  norm_type=cv2.NORM_MINMAX,
                                  dtype=cv2.CV_32F)
            images.append(image)
            classes.append(_class.replace(' ', ''))
    return images, classes


if __name__ == '__main__':
    timestamp = time.strftime('%H_%M_%S')
    default_output_filename = 'output%s.ann' % timestamp
    parser = ArgumentParser(description='Train network')
    parser.add_argument('-o', '--output_weight', type=str,
                        default=default_output_filename,
                        help='File for save trained neural network')
    parser.add_argument('-c', '--configuration_file', type=str, default=None,
                        help='File contained json string with configuration')
    parser.add_argument('-t', '--training_set_file', type=str,
                        help='Training set file')
    parser.add_argument('-x', '--input_size_x', type=int, default=300,
                        help='Input size width of image')
    parser.add_argument('-y', '--input_size_y', type=int, default=300,
                        help='Input size height of image')
    parser.add_argument('-a', '--hidden_size', type=int,
                        help='Hidden size of neural network')

    arguments = parser.parse_args()
    parameters = arguments.configuration_file
    if arguments.configuration_file is not None:
        with open(arguments.configuration_file) as config:
            lines = config.readlines()
            lines_gen = (line.strip() for line in lines)
            json_string = ' '.join(lines_gen)
            parameters = json.loads(json_string)

    size = (arguments.input_size_x, arguments.input_size_y)
    images, classes = load_images(size, arguments.training_set_file)

    input_size = size[0] * size[1]
    output_size = len(set(classes))
    layers = (input_size, arguments.hidden_size, output_size)

    trained_network = train(layers, images, classes, parameters)
    trained_network.save(arguments.output_weight)
    print 'Save trained network in file %s' % arguments.output_weight
