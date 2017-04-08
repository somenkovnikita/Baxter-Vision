from abc import ABCMeta, abstractmethod
from os.path import dirname, join

import cv2
import numpy as np


class ILetterRecognizer:
    """Interface for letter recognize"""

    training_set = list()

    @abstractmethod
    def letter(self, image):
        # type: (ILetterRecognizer, np.array) -> chr
        """Recognize letter on image"""
        raise NotImplementedError('LetterRecognize.letter')

    @abstractmethod
    def letters(self, images):
        # type: (ILetterRecognizer, list(np.array)) -> list
        """Recognize letters on images. This _method may be faster than:
            for img in images: LetterRecognize.letter(img)"""
        raise NotImplementedError('LetterRecognize.letters')

    @staticmethod
    def setup_letters(filename):
        # type: (str) -> None
        """Initialize training set from file in format:
            <image-path> <letter/class>
        """
        ts = ILetterRecognizer.training_set
        basedir = dirname(filename)
        with open(filename) as letters:
            for line in letters:
                image_path, letter = line.split()
                image_path = join(basedir, image_path)
                image = cv2.imread(image_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
                element = image, letter
                ts.append(element)

    __metaclass__ = ABCMeta
