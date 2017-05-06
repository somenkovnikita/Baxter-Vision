from abc import ABCMeta, abstractmethod

import numpy as np

"""Define interface for vision"""


class ICubeDetector:
    """Interface for cube detect"""

    @abstractmethod
    def cubes(self, image):
        # type: (ICubeDetector, np.array) -> list
        """
        Find all cubes on image, return list of rectangles
        
        :param image: input OpenCV image from detecting  
        :return: list of tuple: x1, y1, x2, y2
        """
        pass

    __metaclass__ = ABCMeta


class ILetterRecognizer:
    """Interface for letter recognize"""

    @abstractmethod
    def letter(self, image):
        # type: (ILetterRecognizer, np.array) -> int
        """
        Recognize letter on image
        
        :param image: OpenCV image
        :return: number of class
        """
        pass

    @abstractmethod
    def letters(self, images):
        # type: (ILetterRecognizer, list(np.array)) -> list
        """
        Recognize letters on images. This method may be faster than:
            for img in images: LetterRecognize.letter(img)
            
        :param images: list of OpenCV images 
        :return: numbers of class
        """
        pass

    # @staticmethod
    # def setup_letters(filename):
    #     # type: (str) -> None
    #     """Initialize training set from file in format:
    #         <image-path> <letter/class>
    #     """
    #     ts = ILetterRecognizer.training_set
    #     basedir = dirname(filename)
    #     with open(filename) as letters:
    #         for line in letters:
    #             image_path, class_ = line.split()
    #             image_path = join(basedir, image_path)
    #             image = cv2.imread(image_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    #             element = image, class_
    #             ts.append(element)

    __metaclass__ = ABCMeta
