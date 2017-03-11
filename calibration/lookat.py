import cv2
from baxter import LimbMover

mover = None

dc = 0.05
x = 0.5
y = 0.0

def set_and_log(coord, name):
    coord += dc
    print name, coord

lookAt = False
x_px = 0
y_px = 0
cascade = None

def mouse_callback(event, x, y, flags, param):
    global lookAt, x_px, y_px
    if event == cv2.EVENT_LBUTTONDOWN:
        lookAt = (x, y)
        x_px, y_px = lookAt
        print x, y
    if event == cv2.EVENT_RBUTTONDOWN:
        print x, y



def init(config):
    global mover,x ,y, cascade

    cv2.namedWindow('Aim')
    cv2.setMouseCallback('Aim', mouse_callback)

    cascade = cv2.CascadeClassifier()
    if cascade.load(config):
        print 'Load cascade success'

    mover = LimbMover.LimbMover('left')

    pass


def draw_aim(frame):
    resolution = frame.shape[:2]
    resolution = resolution[1], resolution[0]

    aim_x = 0.5
    aim_y = 0.5

    x = int(aim_x * resolution[0])
    y = int(aim_y * resolution[1])
    p1 = (x, 0)
    p2 = (x, resolution[1])
    p3 = (0, y)
    p4 = (resolution[0], y)
    cv2.line(frame, p1, p2, (255, 255, 255))
    cv2.line(frame, p3, p4, (255, 255, 255))




def lookat(frame):
    cframe = frame.copy()

    draw_aim(cframe)

    global mover, x, dc, y, lookAt, x_px, y_px

    key = cv2.waitKey(10) & 0xff
    if key == ord('w'):
        x += dc
        print 'x =', x
    elif key == ord('s'):
        x -= dc
        print 'x =', x
    elif key == ord('d'):
        y -= dc
        print 'y =', y
    elif key == ord('a'):
        y += dc
        print 'y =', y
    elif key == ord('t'):
        mover.move([0.5, 0.0, 0.1, -3.14, 0.0, 0.0], move=True)
    elif lookAt:
        h, w = cframe.shape[:2]
        x = 0.157 / 172 * (y_px - h / 2)
        y = 0.157 / 172 * (x_px - w / 2)
        print x, y
        lookAt = False

        pose = mover._get_current_pose()
        pose[0] += x
        pose[1] += y
        print pose
        mover.move(pose, move=True)
    elif cascade:
        cubes = cascade.detectMultiScale(cframe)
        h, w = cframe.shape[:2]
        if len(cubes) > 0:
            print 'Found ', len(cubes), 'rects'

            y_px = cubes[0][3] / 2 + cubes[0][1]
            x_px = cubes[0][2] / 2 + cubes[0][0]

            x = 0.157 / 172 * (y_px - h / 2)
            y = 0.157 / 172 * (x_px - w / 2)
            print x, y
            lookAt = False

            pose = mover._get_current_pose()
            pose[0] += x
            pose[1] += y
            print pose
            mover.move(pose, move=True)
        for x, y, w, h in cubes:
            p = x, y
            rect = x + w, y + h
            cv2.rectangle(cframe, p, rect, (255, 255, 255), 2)

    cv2.imshow('Aim', cframe)
    cv2.waitKey(100)


