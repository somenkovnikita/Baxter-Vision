from glob import glob

import cv2 as cv
import numpy as np

from calibration import Calibration

corners_params = Calibration(9, 6, 640, 400)
images = glob('*.png')
for i in images:
    img = cv.imread(i)
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    corners_params.corners(img)
    # cv.imshow('board', img)
    # cv.waitKey(50)
# cv.destroyAllWindows()

mtx, dist, newcameramtx, roi = corners_params.calibration()
x, y, w, h = roi
mapx, mapy = corners_params.remap_params(mtx, dist, newcameramtx)

np.savez('undist_options', x=x, y=y, w=w, h=h,
         mapx=mapx, mapy=mapy, dist=dist,
         newcameramtx=newcameramtx, mtx=mtx)
