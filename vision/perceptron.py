import cv2

from vision.interface import ILetterRecognizer


class PerceptronLetterRecognizer(ILetterRecognizer):

    def __init__(self, config_file):
        default_width, default_height = 25, 25
        self._size = (default_width, default_height)
        self._classes = list()
        self._network = cv2.ANN_MLP()
        self._network.load(config_file)

    # Setup image for neural network
    def prepare_image(self, image):
        prepared = cv2.resize(image, self._size)
        prepared = cv2.cvtColor(prepared, cv2.COLOR_BGR2GRAY)
        prepared = cv2.equalizeHist(prepared)
        return prepared

    def letters(self, images):

        # Setup images
        ready_images = list()
        for image in images:
            ready_images.append(self.prepare_image(image))

        # Recognizing
        try:
            ret, resp = self._network.predict(ready_images)
            predict = resp.argmax(-1)
            print "PerceptronRecognizer: predict" + predict
        except:
            print "PerceptronLetterRecognizer: error predict"
        pass

    def letter(self, image):

        # Setup single image
        prepared = self.prepare_image(image)

        # Recognize single image
        try:
            ret, resp = self._network.predict(prepared)
            predict = resp.argmax(-1)
            print "PerceptronRecognizer: predict" + predict
        except:
            print "PerceptronRecognizer: error predict"
