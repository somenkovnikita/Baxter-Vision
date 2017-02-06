# coding=utf-8
import cv2

from baxter import utils
from baxter.camera import processor

window_title = 'baxter by D.D.M. v.1.0'
param = cv2.cv.CV_CAP_PROP_FPS

# Четкие контуры 88 & 92
# Шум 36 & 44
# Middle 128
min_bound = 88
max_bound = 92
camera_port = 0
camera_width = 1280
camera_height = 720


# ФУНКЦИИ НАСТРОЕК

def set_min(value):
    global min_bound
    min_bound = value
    return min_bound


def set_max(value):
    global max_bound
    max_bound = value
    return max_bound


def set_h(x):
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, x)


def set_w(x):
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, x)


# ГЛАВНЫЙ МЕТОД

capture = cv2.VideoCapture()
capture.open(0)

if not capture.isOpened():
    print 'Error init camera'

# UI
cv2.namedWindow(window_title, flags=cv2.WND_PROP_OPENGL)
# Разрешение
# cv2.createTrackbar("Image Width: ", window_title, camera_width, camera_width, set_w)
# cv2.createTrackbar("Image Height: ", window_title, camera_height, camera_height, set_h)
# Границы для cv2.canny
# cv2.createTrackbar("Min threshold: ", window_title, min_bound, 255, set_min)
# cv2.createTrackbar("Max threshold: ", window_title, max_bound, 255, set_max)

# Шаблоны
files, templates = utils.get_templates(".jpg")

while True:
    # Захват изображения с камеры
    result, frame = capture.read()
    # Зеркальный разворот камеры по горизонтали
    cv2.flip(frame, 1, frame)

    # Если кадр на пустой
    if result:
        # Отображаем результат
        ms = utils.millis()

        # cv2.imshow(window_title, processor.find_image(frame, templates, files))
        cv2.imshow(window_title, processor.find_edges(frame, min_bound, max_bound))
        # cv2.imshow(window_title, frame)
        delta = utils.millis() - ms
        print "Time delta: %d" % delta

    # Escape
    if cv2.waitKey(1) == 27:
        break

# Освобождение ресурсов
capture.release()
cv2.destroyAllWindows()
