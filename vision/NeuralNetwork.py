import cv2
import numpy as np

from BaseNeuralNetwork import BaseNeuralNetwork


class NeuralNetwork(BaseNeuralNetwork):
    def __init__(self, sizes=None, config_file=None):
        self._network = cv2.ANN_MLP()
        self._classes = dict()
        if sizes is not None:
            np_size = np.array(sizes)
            self._network.create(np_size)
        elif config_file is not None:
            self._network.load(config_file)

    def train(self, images, classes):
        # type: (NeuralNetwork, list, list) -> None
        pass

    @staticmethod
    def _prepare_set(images):
        # type: (list) -> np.array
        to_gray = lambda img: cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grays = (to_gray(img) for img in images)
        

        for image in images:

        pass


