import cv2 as cv
import numpy as np
from Image import Video_Camera


video_stream = cv.VideoCapture(1)
cam = Video_Camera(640, 480)
mapx = np.load("mapx.npy")
mapy = np.load("mapy.npy")

print (mapx)

while True:
    frame = cam.get_frame(video_stream)
    undist_frame = cam.undistort_img(frame, mapx, mapy)
    cv.imshow("q", undist_frame)
    cv.imshow("qweqweqwe", frame)
    if cv.waitKey(1) & 0xFF == ord("q"):
        break