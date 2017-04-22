# -*- coding: utf-8 -*-

import cv2

from interface import ILetterRecognizer


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
