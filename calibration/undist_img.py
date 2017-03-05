import cv2 as cv
import numpy as np
import rospy

from src.camera import BaxterCamera

rospy.init_node('calib')

video_stream = BaxterCamera('left_hand')

undist_options = np.load('undist_options.npz')

mapx = undist_options['mapx']
mapy = undist_options['mapy']
x = undist_options['x']
y = undist_options['y']
w = undist_options['w']
h = undist_options['h']
dist = undist_options['dist']
newcameramtx = undist_options['newcameramtx']
mtx = undist_options['mtx']

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