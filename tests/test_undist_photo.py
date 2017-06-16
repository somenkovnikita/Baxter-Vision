import cv2
import numpy as np
import rospy
from tools import utils
from baxter import Camera

# OpenCV Baxter make photos test by Ivan. 2017(c)

rospy.init_node("undist_photo")

TEST_NAME = "OpenCV Baxter make photos"
photo_count = 1

print TEST_NAME + " >> Init..."
bax_camera = Camera(utils.LEFT_BAXTER_CAMERA)
print TEST_NAME + " >> OK"

x = np.load("x.npy")
y = np.load("y.npy")
w = np.load("w.npy")
h = np.load("h.npy")
dist = np.load("dist.npy")
new_camera_mtx = np.load("newcameramtx.npy")
mtx = np.load("mtx.npy")

cv2.namedWindow(TEST_NAME, flags=cv2.WND_PROP_OPENGL)

while True:

    # If frame available
    if bax_camera.read_frame():
        # Show frame
        frame = bax_camera.get_frame()
        undist_frame = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)
        undist_frame = undist_frame[y:y+h, x:x+w]
        cv2.imshow(TEST_NAME, undist_frame)

    # Press "W" to write a frame
    if cv2.waitKey(1) & 0xFF == ord("w"):
        cv2.imwrite(str(photo_count) + ".png", undist_frame)
        print photo_count

    # Escape
    elif cv2.waitKey(1) == utils.ESCAPE_KEY:
        print TEST_NAME + " >> Exit"
        break

# Free all
print TEST_NAME + " >> Free"
bax_camera.free()
