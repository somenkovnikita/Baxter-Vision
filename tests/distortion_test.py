import numpy
import cv2
from tools import utils


# Images distortion.
# We can use this script for generate
# images for neural network

def im_distortion(folder, color):
    # Templates
    im_data = utils.read_images(folder)
    i = 0

    (image, cls) = im_data[0]
    cv2.imshow("Image distortion test(Image # %d)" % i, image)

    for item in im_data:
        (image, cls) = item

        # Perspective
        # http://docs.opencv.org/trunk/da/d6e/tutorial_py_geometric_transformations.html
        rows, cols, ch = image.shape
        pts1 = numpy.float32([[16, 16], [100, 164], [10, 140]])
        pts2 = numpy.float32([[8, 10], [100, 164], [10, 170]])
        m = cv2.getAffineTransform(pts1, pts2)
        result = cv2.warpAffine(image, m, (cols, rows))

        # Rotation
        # image_center = tuple(numpy.array(im_bw.shape) / 3.14)
        # rot_mat = cv2.getRotationMatrix2D(image_center, 90, 0.75)
        # result = cv2.warpAffine(im_bw, rot_mat, im_bw.shape, flags=cv2.INTER_LINEAR)

        result = cv2.cvtColor(result, color)

        cv2.imshow("Image distortion test(Image # %d)" % i, result)
        i += 1


while True:

    im_distortion(utils.NEURAL_NET_DIR, cv2.COLOR_BGR2GRAY)
    # im_distortion("gray", cv2.COLOR_BGR2GRAY)
    # im_distortion("hls", cv2.COLOR_BGR2HLS)
    # im_distortion("lab", cv2.COLOR_BGR2LAB)
    # im_distortion("xyz", cv2.COLOR_BGR2XYZ)
    # im_distortion("yuv", cv2.COLOR_BGR2YUV)
    # im_distortion("pers4", cv2.COLOR_BGR2GRAY)
    # Escape
    if cv2.waitKey(1) == utils.ESCAPE_KEY:
        break

cv2.destroyAllWindows()