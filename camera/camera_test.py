# coding=utf-8
import cv2

from baxter import camera

# coding=utf-8
# Здесь можно тестировать любой код

window_title = "baxter camera Test"
cam = camera.camera("right_hand_camera", (640, 480))
cam.setup_camera()
capture = cv2.VideoCapture()
capture.open(0)

while True:
    # Захват изображения с камеры
    result, frame = capture.read()
    # Зеркальный разворот камеры по горизонтали
    cv2.flip(frame, 1, frame)

    # Если кадр на пустой
    if result:
        cv2.imshow(window_title, cam.get_image())

    # Escape
    if cv2.waitKey(1) == 27:
        break

# Освобождение ресурсов
capture.release()
cv2.destroyAllWindows()
