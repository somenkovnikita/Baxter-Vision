import cv2
import LocalCamera


class CameraForTestAlgorithm:
    def __init__(self):
        self.camera = LocalCamera.LocalCamera((1270, 800))

    def run(self, image_processor):
        while True:
            image = self.camera.get_image()

            processed_image = image_processor(image)

            cv2.imshow("real-time camera", processed_image)
            key = cv2.waitKey(10)

            if key & 0xFF == ord("q"):
                return
