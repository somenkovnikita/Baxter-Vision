import cv2
import tools.CameraForTestAlgorithm
import random


def process(image):
    imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh,
        cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    for i, contour in enumerate(contours):
        r = random.randint(100, 255)
        b = random.randint(100, 255)
        g = random.randint(100, 255)
        cv2.drawContours(image, contours, i, (r, b, g), 1)

    return image

camera = tools.CameraForTestAlgorithm()
camera.run(process)
