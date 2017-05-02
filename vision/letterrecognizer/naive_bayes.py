import cv2
import numpy as np
from sklearn.externals import joblib

from interface import ILetterRecognizer


class NaiveBayes(ILetterRecognizer):
    def __init__(self, config_file):
        self._model = joblib.load(config_file)

    def letters(self, images):
        pass

    def letter(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.equalizeHist(image)

        image = cv2.resize(image, (50, 50))
        image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 29, 0)
        
        img = np.array([image.flatten()/255.0])
        return self._model.predict(img)

    def preproc(self, image):
        pass
