import cv2
import numpy as np
from sklearn.externals import joblib

from interface import ILetterRecognizer


class SVMLetterRecognizer(ILetterRecognizer):
    """
    Support Vector Machine
    See description of override method in interface
    """
    image_size = 25, 25

    def __init__(self, config_file):
        # type: (SVMLetterRecognizer, str) -> None
        """
        Load pre trained config
        
        :param config_file: file path with pre trained model
        """
        self._svm_model = joblib.load(config_file)

    def letters(self, images):
        processing = SVMLetterRecognizer.preprocessing
        processed_images = np.array(map(processing, images))
        return self._svm_model.predict(processed_images)

    def letter(self, image):
        processed_image = SVMLetterRecognizer.preprocessing(image)
        return self._svm_model.predict(np.array([processed_image]))[0]

    @staticmethod
    def preprocessing(image):
        # type: (np.array) -> np.array
        """
        Prepare image to recognize
         
        :param image: OpenCV image to prepare
        :return: prepared OpenCV image 
        """
        if image.shape[2] == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.equalizeHist(image)
        image = cv2.resize(image, SVMLetterRecognizer.image_size)
        return image.flatten().astype(float) / 255.0


class TemplateLetterRecognizer(ILetterRecognizer):
    default_width, default_height = 25, 25
    default_size = (default_width, default_height)
    default_method = cv2.cv.CV_TM_SQDIFF_NORMED
    default_threshold = 0.5
    methods = {
        # read OpenCV docs for description
        cv2.cv.CV_TM_CCORR_NORMED,
        cv2.cv.CV_TM_CCOEFF_NORMED,
        cv2.cv.CV_TM_SQDIFF_NORMED
    }

    def __init__(self, size=default_size, method=default_method, threshold=default_threshold):
        # TODO: try for different parameters value in diploma
        self._size = size
        self._method = method
        self._threshold = threshold
        self._templates = list()
        self._classes = list()
        self._collect_unique_templates()

    def letter(self, image):
        candidates = list()
        prepared = cv2.resize(image, self._size)
        prepared = cv2.cvtColor(prepared, cv2.COLOR_BGR2GRAY)
        prepared = cv2.equalizeHist(prepared)
        for i, template in enumerate(self._templates):
            match = self._match_template(prepared, template)
            if match >= self._threshold:
                candidates.append((i, match))
        if candidates:
            idx = max(candidates, key=lambda x: x[1])[0]
            return self._classes[idx]

    def letters(self, images):
        return [self.letter(img) for img in images]

    def _match_template(self, image, template):
        args = image, template, self._method
        match = cv2.matchTemplate(*args)
        min_max = cv2.minMaxLoc(match)
        min_val, max_val = min_max[:2]
        if self._method == cv2.TM_SQDIFF_NORMED:
            return 1.0 - min_val
        return max_val

    def _collect_unique_templates(self):
        unique_letter = set()
        training_set = ILetterRecognizer.training_set
        for template, letter in training_set:
            if letter in unique_letter:
                continue
            unique_letter.add(letter)
            resized = cv2.resize(template, self._size)
            template = cv2.equalizeHist(resized)
            self._templates.append(template)
            self._classes.append(letter)


class PerceptronLetterRecognizer(ILetterRecognizer):
    def __init__(self, config_file):
        default_width, default_height = 25, 25
        self._size = (default_width, default_height)
        self._classes = list()
        self._network = cv2.ANN_MLP()
        self._network.load(config_file)

    # About _network.predict()
    # Predicts responses for input samples.
    # The method returns a dummy value which should be ignored.
    # If you are using the default cvANN_MLP::SIGMOID_SYM activation function with the default parameter
    # values fparam1=0 and fparam2=0 then the function used is y = 1.7159*tanh(2/3 * x),
    # so the output will range from [-1.7159, 1.7159], instead of [0,1].


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
