from calibration.Calibration import Calibration
import numpy as np
import cv2 as cv
from glob import glob


corners_params = Calibration(9, 6, 640, 400)
images = glob('*.png')
for i in images:
    img = cv.imread(i)
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    corners_params.corners(img)
    cv.imshow('board', img)
    cv.waitKey(50)
cv.destroyAllWindows()

mtx, dist, newcameramtx, roi = corners_params.calibration()
x, y, w, h = roi
mapx, mapy = corners_params.remap_params(mtx, dist, newcameramtx)

np.save('x.npy', x)
np.save('y.npy', y)
np.save('w.npy', w)
np.save('h.npy', h)
np.save('mapx.npy', mapx)
np.save('mapy.npy', mapy)
np.save('dist.npy', dist)
np.save('newcameramtx.npy', newcameramtx)
np.save('mtx.npy', mtx)
