import cv2 as cv
import numpy as np
import rospy

from src.camera import BaxterCamera

rospy.init_node('calib')

video_stream = BaxterCamera('left_hand')

mapx = np.load('mapx.npy')
mapy = np.load('mapy.npy')

x = np.load('x.npy')
y = np.load('y.npy')
w = np.load('w.npy')
h = np.load('h.npy')
dist = np.load('dist.npy')
newcameramtx = np.load('newcameramtx.npy')
mtx = np.load('mtx.npy')
p = False
while True:
    frame = video_stream.get_frame()
    dst = cv.undistort(frame, mtx, dist, None, newcameramtx)
    undist_frame = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)
    dst = dst[y:y+h, x:x+w]
    if p is False:
        print dst.shape, frame.shape, x, y, w, h
        p = True
    cv.imshow('undist_frame', undist_frame)
    cv.imshow('original_frame', frame)
    cv.imshow('dst', dst)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break