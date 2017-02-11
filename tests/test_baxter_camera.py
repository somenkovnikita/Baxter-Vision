import cv2
from tools import utils
from camera import BaxterCamera

# Baxter camera test by D.D.M. 2017(c)

TEST_NAME = "OpenCV Baxter camera test"

print TEST_NAME + " >> Init..."
bax_camera = BaxterCamera(utils.HEAD_BAXTER_CAMERA)
# local_camera.set_color(cv2.COLOR_BGR2HLS_FULL)
print TEST_NAME + " >> OK"

cv2.namedWindow(TEST_NAME, flags=cv2.WND_PROP_OPENGL)

while True:

    # If frame available
    if bax_camera.read_frame():
        # Show frame
        frame = bax_camera.get_frame()
        cv2.imshow(TEST_NAME, frame)

    # Escape
    if cv2.waitKey(1) == utils.ESCAPE_KEY:
        print TEST_NAME + " >> Exit"
        break

# Free all
print TEST_NAME + " >> Free"
bax_camera.free()

