import os
import sys

back = os.path.abspath('..')
sys.path.append(back)

import argparse
import rospy
from core import Camera
from Calibration import Calibration


"""
Make dataset for calibration
Pattern here: http://docs.opencv.org/2.4/_downloads/pattern.png
"""


def make_dataset(out_dir, camera_name):
    rospy.init_node('make_dataset')
    base = os.path.abspath(out_dir)

    camera = Camera(camera_name)
    corners_params = Calibration(9, 6, 800, 600)
    photo_num = 0

    while True:
        frame = camera.get_frame()
        corners = corners_params.find_corners(frame)
        drawed_frame = frame.copy()
        if corners:
            corners_params.drawing_corners(drawed_frame)
        cv2.imshow('q', drawed_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and corners:
            out_file = str(photo_num) + '.png'
            out_path = os.path.join(base, out_file)
            cv2.imwrite(out_path, frame)
            print 'Save:', out_path
            photo_num += 1
        elif key == ord('q'):
            break


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--out_dir", type=str, default='.',
                        help="Output dir for images")
    parser.add_argument("-c", "--camera_name", type=str, default='left_hand',
                        help="Name of Baxter camera: left_hand, right_hand")

    args = parser.parse_args()

    make_dataset(args.out_dir, args.camera_name)
