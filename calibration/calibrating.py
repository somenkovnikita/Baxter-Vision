from Calibration import Calibration
import numpy as np
import cv2 as cv


corners_params = Calibration(7, 7, 640, 480)
for i in range(1, 29):
    img = cv.imread(str(i)+".png")
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    corners_params.corners(img)
    print(i)

mtx, dist, newcameramtx = corners_params.calibration()
mapx, mapy = corners_params.remap_params(mtx, dist, newcameramtx)

np.save("mapx.npy", mapx)
np.save("mapy.npy", mapy)