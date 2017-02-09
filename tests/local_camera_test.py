import cv2
from tools import utils
from camera import LocalCamera

# Simple camera test for OpenCV D.D.M. 2017(c)

TEST_NAME = "OpenCV local camera test"

print TEST_NAME + " >> Init..."
local_camera = LocalCamera(0, (640, 480))
local_camera.set_fps(60)
# local_camera.set_color(cv2.COLOR_BGR2HLS_FULL)
print TEST_NAME + " >> OK"

cv2.namedWindow(TEST_NAME, flags=cv2.WND_PROP_OPENGL)

while True:

    # If frame available
    if local_camera.read_frame():
        # Show frame
        frame = local_camera.get_frame()
        cv2.imshow(TEST_NAME, frame)

    # Escape
    if cv2.waitKey(1) == utils.ESCAPE_KEY:
        print TEST_NAME + " >> Exit"
        break

# Free all
print TEST_NAME + " >> Free"
local_camera.free()
cv2.destroyAllWindows()
