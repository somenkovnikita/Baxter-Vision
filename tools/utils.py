import cv2
import os

NEURAL_NET_DIR = "../assets/neural/training_set"
NEURAL_SET_FILE = "../config/training_set.txt"
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
        # FIXME: What's first? Resize || normalization?
        # Resizing(for neural network)
        rs_image = cv2.resize(image, (width, height))
        # Normalization
        norm_image = cv2.normalize(rs_image, alpha=-1, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        result.append((norm_image, fl))
    return result
