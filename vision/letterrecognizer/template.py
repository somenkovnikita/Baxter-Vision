# -*- coding: utf-8 -*-


import cv2

from interface import ILetterRecognizer


class TemplateLetterRecognizer(ILetterRecognizer):
    default_width, default_height = 50, 50
    default_size = (default_width, default_height)
    default_method = cv2.CV_TM_CCORR_NORMED
    default_threshold = 0.8

    def __init__(self, size=default_size, method=default_method, threshold=default_threshold):
        # TODO: try for different parameters value in diploma
        self.size = size
        self.method = method
        self.threshold = threshold
        self.templates = list()
        self.classes = list()
        self._collect_unique_templates()

    def letter(self, image):
        candidates = list()
        for i, template in enumerate(self.templates):
            match = self._math_template(image, template)
            max_val = max(match)
            if max_val >= self.threshold:
                candidates.append((i, max_val))
        idx = max(candidates, key=lambda x: x[1])[0]
        return self.classes[idx]

    def letters(self, images):
        return [self.letter(img) for img in images]

    def _math_template(self, image, template):
        args = image, template, self.method
        match = cv2.matchTemplate(*args)
        min_max = cv2.minMaxLoc(match)
        min_val, max_val = min_max[:2]
        if self.method == cv2.TM_SQDIFF_NORMED:
            return 1.0 - min_val
        return max_val

    def _collect_unique_templates(self):
        unique_letter = set()
        training_set = ILetterRecognizer.training_set
        for template, letter in training_set:
            if letter in unique_letter:
                continue
            resized = cv2.resize(template, self.size)
            self.templates.append(resized)
            self.classes.append(letter)
