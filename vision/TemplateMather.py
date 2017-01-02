import numpy
import cv2


class TemplateMatcher:

    methods = [cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF_NORMED]

    def __init__(self, template_image, method=None):
        self.method = 0 if method is None else method
        self.template = template_image
        self.image = numpy.zeros((1280, 800, 3), numpy.uint8)
        self.template_size = template_image.shape[:2][::-1]

    def match(self, eps=None):
        eps = 0.75 if eps is None else float(eps)
        method = self.methods[self.method]
        res = cv2.matchTemplate(self.image, self.template, method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            top_left = min_loc
        else:
            top_left = max_loc
        if max_val < eps:
            return 0, 0, 0, 0
        w, h = self.template_size
        bottom_right = (top_left[0] + w, top_left[1] + h)
        return top_left[0], top_left[1], bottom_right[0], bottom_right[1]

    def set_method(self, method_index):
        self.method = method_index

    def set_image(self, image):
        self.image = image
