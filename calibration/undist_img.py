import cv2 as cv
import numpy as np
from Image import Video_Camera


video_stream = cv.VideoCapture(0)
cam = Video_Camera(800, 600)
mapx = np.load('mapx.npy')
mapy = np.load('mapy.npy')

x = np.load('x.npy')
y = np.load('y.npy')
w = np.load('w.npy')
h = np.load('h.npy')

while True:
    frame = cam.get_frame(video_stream)
    undist_frame = cam.undistort_img(frame, mapx, mapy)
    undist_frame = undist_frame[y:y+h, x:x+w]
    cv.imshow('undist_frame', undist_frame)
    cv.imshow('original_frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break