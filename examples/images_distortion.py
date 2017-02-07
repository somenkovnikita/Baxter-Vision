import os
import cv2
import numpy

import baxter.utils

# Images distortion.
# We can use this script for generate
# images for neural network
# D.D.M. 2016(C)
print "Images distortion example"


def im_distortion(file_dir):
    # Tepmlates
    os.makedirs(file_dir)
    files, templates = baxter.Camera.utils.get_templates()
    i = 0
    for temp in templates:
        filename = os.path.splitext(files[i])[0]
        # new_img = cv2.cvtColor(temp, tp)
        new_filename = file_dir + "/" + filename + ".jpg"
        # (thresh, im_bw) = cv2.threshold(new_img, 128, 255, cv2.THRESH_BINARY)

        # Perspective
        # http://docs.opencv.org/trunk/da/d6e/tutorial_py_geometric_transformations.html
        rows, cols, ch = temp.shape
        pts1 = numpy.float32([[16, 16], [100, 164], [10, 140]])
        pts2 = numpy.float32([[8, 10], [100, 164], [10, 170]])
        m = cv2.getAffineTransform(pts1, pts2)
        result = cv2.warpAffine(temp, m, (cols, rows))

        # Rotation
        # image_center = tuple(numpy.array(im_bw.shape) / 3.14)
        # rot_mat = cv2.getRotationMatrix2D(image_center, 90, 0.75)
        # result = cv2.warpAffine(im_bw, rot_mat, im_bw.shape, flags=cv2.INTER_LINEAR)

        cv2.imwrite(new_filename, result)
        print "Image write: " + new_filename
        i += 1

        # fuck_up("hsv", cv2.COLOR_BGR2HSV)
        # fuck_up("gray", cv2.COLOR_BGR2GRAY)
        # fuck_up("hls", cv2.COLOR_BGR2HLS)
        # fuck_up("lab", cv2.COLOR_BGR2LAB)
        # fuck_up("xyz", cv2.COLOR_BGR2XYZ)
        # fuck_up("yuv", cv2.COLOR_BGR2YUV)
        # fuck_up("pers4", cv2.COLOR_BGR2GRAY)
