import cv2 as cv
from Image import Video_Camera
from Calibration import Calibration


video_stream = cv.VideoCapture(1)
cam = Video_Camera(640, 480)
corners_params = Calibration(7, 7, 480, 320)
photo_num = 1

while True:
    frame = cam.get_frame(video_stream)
    corners = corners_params.find_corners(frame)
    cv.imshow("q", frame)
    if cv.waitKey(1) & 0xFF == ord("s"):
        if corners:
            cv.imwrite(str(photo_num) + ".png", frame)
            print(str(photo_num)+" photo.")
            photo_num += 1
    elif cv.waitKey(1) & 0xFF == ord("q"):
        break


