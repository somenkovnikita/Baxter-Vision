import cv2

cascade = None
i = 0


def init(config):
    global cascade

    cascade = cv2.CascadeClassifier()
    if cascade.load(config):
        print "Load cascade success"
        return

    raise Exception("Cascade not load")


def run(frame):
    global i

    image = frame.copy()
    cubes = cascade.detectMultiScale(image)
    if len(cubes) > 0:
        print "Found ", len(cubes), "rects"
    for x, y, w, h in cubes:
        p = x, y
        rect = x + w, y + h
        cv2.rectangle(image, p, rect, (255, 255, 255), 2)
        cb = cv2.flip(frame[y:y+h, x:x+w], 1)

        cv2.imwrite(str(i) + '.jpg', cb)
        i += 1
        
    cv2.imshow("Cascade", image)
    # 
    key = cv2.waitKey(100) & 0xFF
    # if key == ord('s'):
    #     cv2.imwrite(str(i) + '.png', image)
    #     i += 1
