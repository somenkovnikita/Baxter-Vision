import cv2

cascade = None


def init(config):
    global cascade

    cascade = cv2.CascadeClassifier()
    if cascade.load(config):
        print "Load cascade success"
        return

    raise Exception("Cascade not load")


def run(frame):
    image = frame.copy()
    cubes = cascade.detectMultiScale(image)
    if len(cubes) > 0:
        print "Found ", len(cubes), "rects"
    for x, y, w, h in cubes:
        p = x, y
        rect = x + w, y + h
        cv2.rectangle(image, p, rect, (255, 255, 255), 2)

    cv2.imshow("Cascade", image)
