import cv2
import json
import numpy as np
import NeuralNetworkParams


class NeuralNetwork:
    """
    Sweet wrapper for opencv multi-layer perceptrons, oriented for training_set
    """
    def __init__(self, layers=None, filename=None):
        # type: (tuple, str) -> None
        """
        :param layers: Set structure neural network
        :param filename: Set file for load trained network
        """
        self.classes = dict()
        if filename:
            with open(filename + ".cl") as cl_file:
                line = cl_file.readline().strip()
                self.classes = json.loads(line)
            self.layers_size = self.classes["LayerSize"]
            self.classes.pop("LayerSize")
            self.network = cv2.ANN_MLP()
            self.network.load(filename)
        elif layers:
            self.layers_size = layers
            self.network = cv2.ANN_MLP(np.array(layers))
        else:
            raise Exception("Layers or filename is require")

    def train(self, input_images, output_classes, parameters=NeuralNetworkParams()):
        # type: (list, list, NeuralNetworkParams) -> int
        """
        :param input_images: Input dataset of training_set
        :param output_classes: Output dataset of correct class
        :param parameters: Train parameters
        :return: Count of iteration of train
        """
        flatten_images = [img.flatten() for img in input_images]
        input_matrix = np.asarray(flatten_images, np.float)

        output_size = self.layers_size[-1]
        self.classes = {i: c for i, c in enumerate(set(output_classes))}
        flatten_classes = []
        indices = {v: k for k, v in self.classes.items()}
        for class_ in output_classes:
            row = np.zeros(output_size) - 1.0
            row[indices[class_]] = 1.0
            flatten_classes.append(row)
        output_matrix = np.asarray(flatten_classes, np.float)

        sample_weights = None
        iteration = self.network.train(
            input_matrix, output_matrix, sample_weights,
            params=parameters.get())
        return iteration

    def classify(self, image):
        # type: (np.array) -> any
        """
        :param image: Input image
        :return: output_set: Output dataset, matrix: class_size * count_class
        """
        flatten = image.flatten()
        ret_value, output = self.network.predict(flatten)
        if ret_value is not 0:
            raise Exception("Error classify")
        return self.classes[np.argmax(output)]

    def save(self, filename):
        # type: (str) -> None
        """
        :param filename: Set file for save trained network
        :return:
        """
        self.classes["LayerSize"] = self.layers_size
        with open(filename + ".cl", "w") as cl_file:
            classes = json.dumps(self.classes)
            cl_file.write(classes)
        self.network.save(filename)
        self.classes.pop("LayerSize")
