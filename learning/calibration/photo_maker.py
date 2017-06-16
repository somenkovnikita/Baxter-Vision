import ConfigParser
import os

import cv2

"""Module for real_time.py script for make dataset for calibration"""


basedir = ""
photo_num = 0
chess_rows = 9
chess_cols = 6
chess_size = (chess_rows, chess_cols)


def init(ini_config_fn):
    global chess_rows, chess_cols, chess_size

    parser = ConfigParser.ConfigParser()
    parser.read(ini_config_fn)

    if parser.has_section("common"):
        chess_rows = int(parser.get("common", "chess_rows"))
        chess_cols = int(parser.get("common", "chess_cols"))
        chess_size = (chess_rows, chess_cols)

    basedir = parser.get("photo_maker", "output_dir")

    if not os.path.exists(basedir):
        os.mkdir(basedir)


def maker(frame):
    global chess_size, photo_num, basedir

    frame_chess = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, chess_size, None)
    cv2.drawChessboardCorners(frame_chess, chess_size, corners, ret)

    cv2.imshow("calibration::photo_maker", frame_chess)

    if cv2.waitKey(10) & 0xFF == ord("s") and ret:
        image_fn = str(photo_num) + ".jpg"
        path = os.path.join(basedir, image_fn)
        cv2.imwrite(path, frame)
        print "Save new photo:", path
        photo_num += 1
