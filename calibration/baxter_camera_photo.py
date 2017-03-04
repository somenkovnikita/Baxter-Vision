import cv2
import rospy
from camera import BaxterCamera

from calibration import Calibration

"""
Make dataset for calib camera of Baxter Robot
"""

rospy.init_node('calibration')

camera = BaxterCamera('left_hand')
corners_params = Calibration(9, 6, 800, 600)
photo_num = 0

while True:
    frame = camera.get_frame()
    corners = corners_params.find_corners(frame)
    drawed_frame = frame.copy()
    if corners:
        corners_params.drawing_corners(drawed_frame)
    cv2.imshow('q', drawed_frame)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        if corners:
            cv2.imwrite(str(photo_num) + '.png', frame)
            print(str(photo_num)+' photo.')
            photo_num += 1
    elif cv2.waitKey(1) & 0xFF == ord('q'):
        break

