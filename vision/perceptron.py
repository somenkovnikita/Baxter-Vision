import cv2

from vision.interface import ILetterRecognizer


class PerceptronLetterRecognizer(ILetterRecognizer):
    def __init__(self, config_file):
        self._network = cv2.ANN_MLP()
        self._network.load(config_file)
        self._classes = dict()
        self._size_image = None

    def letters(self, images):
        pass

    def letter(self, image):
        pass
