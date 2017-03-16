import cv2
import os
import numpy

LEFT_BAXTER_CAMERA = "left_hand"
RIGHT_BAXTER_CAMERA = "right_hand"
HEAD_BAXTER_CAMERA = "head"
NEURAL_NET_DIR = "../assets/neural/training_set"
NEURAL_SET_FILE = "../config/training_set.txt"
RED_COLOR = (0, 0, 255)
ESCAPE_KEY = 27


# Reading images from folder
def read_images(folder):
    # type: (str) -> list(tuple)
    ext = {".jpg", ".jpeg", ".png", ".bmp"}
    files = []
    dirs = os.listdir(folder)

    for d in dirs:
        # Reading each folder
        # One folder == One class
        path = os.path.join(folder, d)
        if os.path.isdir(path):
            files_list = os.listdir(path)
            for f in files_list:
                for xt in ext:
                    # Check file extension
                    full_path = os.path.join(path, f)
                    if full_path.endswith(xt):
                        # Write path + class name
                        files.append((full_path, d))
    # Loading images
    im_data = []
    for (fl, cls) in files:
        image = cv2.imread(fl, cv2.IMREAD_COLOR)
        im_data.append((image, cls))
    return im_data


def setup_images(im_data, (width, height)):
    result = []

    for im in im_data:
        (image, fl) = im
        # Resizing(for neural network)
        rs_image = cv2.resize(image, (width, height))
        # Normalization
        norm_image = cv2.normalize(rs_image, alpha=-1, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        result.append((norm_image, fl))
    return result


# OpenCV extensions for camera

# Search on frame
def find_image(frame, templates, files):
    i = 0
    fm = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    for image in templates:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        x, y, width, height = find_template(fm, image)
        if x > 0 and y > 0 and width > 0 and height > 0:
            # Draw rectangle on image
            cv2.rectangle(frame, (x, y), (x + width, y + height), RED_COLOR)
            print "Found: "
            print files[i]
        i += 1

    return frame


# Search abc
def find_abc(frame, templates, files, bound_low, bound_up):
    i = 0
    frame = find_edges(frame, bound_low, bound_up)
    for image in templates:
        image = find_edges(image, bound_low, bound_up)
        x, y, width, height = find_template(frame, image)
        if x > 0 and y > 0 and width > 0 and height > 0:
            # Draw rectangle on image
            cv2.rectangle(frame, (x, y), (x + width, y + height), RED_COLOR)
            print "Found: "
            print files[i]
        i += 1

    return frame


# Search for image borders
def find_edges(frame, bound_low, bound_up):
    fm = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(fm, bound_low, bound_up, apertureSize=3)
    return apply_morph_filter(edges)


# Morphology filter
def apply_morph_filter(gray_frame):
    kernel = numpy.ones((3, 5))
    return cv2.morphologyEx(gray_frame, cv2.MORPH_GRADIENT, kernel)


# Search image on frame
def find_template(src_image, tmp_image):
    width, height = tmp_image.shape[::]
    res = cv2.matchTemplate(src_image, tmp_image, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    position = max_loc

    # IMHO 0.65-0.75 best values
    if max_val > 0.70:
        # x, y, width, height
        return position[0], position[1], width, height

    return 0, 0, 0, 0
