# coding=utf-8
import cv2
from camera import processor

# Simple camera test for OpenCV
# D.D.M. 2016(C)
window_title = "OpenCV camera test by D.D.M."
param = cv2.cv.CV_CAP_PROP_FPS

# Best values 88 & 92
# Noise 36 & 44
# Middle 128
min_bound = 88
max_bound = 92
camera_port = 0
camera_width = 1280
camera_height = 720

# SETTINGS FOR CAMERA

def set_min(value):
    global min_bound
    min_bound = value
    return min_bound


def set_max(value):
    global max_bound
    max_bound = value
    return max_bound


def set_h(x):
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, x)


def set_w(x):
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, x)


# CAPTURE VIDEO

capture = cv2.VideoCapture()
capture.open(0)

if not capture.isOpened():
    print "Error camera initialization"

# UI
cv2.namedWindow(window_title, flags=cv2.WND_PROP_OPENGL)
# Resolution
# cv2.createTrackbar("Image Width: ", window_title, camera_width, camera_width, set_w)
# cv2.createTrackbar("Image Height: ", window_title, camera_height, camera_height, set_h)
# Borders for cv2.canny
# cv2.createTrackbar("Min threshold: ", window_title, min_bound, 255, set_min)
# cv2.createTrackbar("Max threshold: ", window_title, max_bound, 255, set_max)

while True:
    # Capture frame from camera
    result, frame = capture.read()
    # Fix mirror effect on frame
    cv2.flip(frame, 1, frame)

    # If frame not empty
    if result:

        # cv2.imshow(window_title, processor.find_image(frame, templates, files))

        # Show borders
        #cv2.imshow(window_title, processor.find_edges(frame, min_bound, max_bound))

        # Show raw frame
        cv2.imshow(window_title, frame)

    # Escape
    if cv2.waitKey(1) == 27:
        break

# Free all
capture.release()
cv2.destroyAllWindows()
