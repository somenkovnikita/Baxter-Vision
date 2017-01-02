import numpy as np
import cv2
import tools.CameraForTestAlgorithm


def proc(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    corners = cv2.goodFeaturesToTrack(gray, 25, 0.01, 10)
    corners = np.int0(corners)

    for i in corners:
        x, y = i.ravel()
        cv2.circle(image, (x, y), 3, 255, -1)

    return image

camera = tools.CameraForTestAlgorithm()
camera.run(proc)
