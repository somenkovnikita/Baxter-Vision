import tools.LocalCamera
import cv2

camera = tools.LocalCamera((1280, 720))

cv2.namedWindow('LocalCamera')

while True:
    image = camera.get_image()

    cv2.imshow('LocalCamera', image)

    if cv2.waitKey(50) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()