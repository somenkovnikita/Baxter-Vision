import cv2 as cv

from Calibration import Calibration
from Image import Video_Camera

video_stream = cv.VideoCapture(0)
cam = Video_Camera(800, 600)
corners_params = Calibration(9, 6, 800, 600)
photo_num = 1

while True:
    frame = cam.get_frame(video_stream)
    frame_to_save = cam.get_frame(video_stream)
    corners = corners_params.find_corners(frame)
    if corners:
        corners_params.drawing_corners(frame)
        cv.imshow('q', frame)
    else:
        cv.imshow('q', frame)
    if cv.waitKey(1) & 0xFF == ord('s'):
        if corners:
            cv.imwrite(str(photo_num) + '.png', frame_to_save)
            print(str(photo_num)+' photo.')
            photo_num += 1
    elif cv.waitKey(1) & 0xFF == ord('q'):
        break


