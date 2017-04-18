import cv2
from baxter import handmover

import baxter_interface

# import baxter_external_device
#
# from baxter_interface import CHECK_VERSION

mover = None

dc = 0.03
x = 0.5
y = 0.0
z = 0.1


def set_and_log(coord, name):
    coord += dc
    print name, coord


lookAt = False
x_pcdx = 0
y_px = 0
cascade = None

grip = None


def mouse_callback(event, x, y, flags, param):
    global lookAt, x_px, y_px
    if event == cv2.EVENT_LBUTTONDOWN:
        lookAt = (x, y)
        x_px, y_px = lookAt
        print x, y
    if event == cv2.EVENT_RBUTTONDOWN:
        print x, y


def init(config):
    global mover, x, y, cascade, grip

    cv2.namedWindow('Aim')
    cv2.setMouseCallback('Aim', mouse_callback)

    grip = baxter_interface.Gripper('left')
    grip.reset_custom_state()

    cascade = cv2.CascadeClassifier()
    if cascade.load(config):
        print 'Load cascade success'

    mover = HandMover.HandMover('left')

    pass


aim_x = 0.56
aim_y = 0.45


def draw_aim(frame):
    global aim_x, aim_y
    resolution = frame.shape[:2]
    resolution = resolution[1], resolution[0]

    print aim_x, aim_y

    x = int(aim_x * resolution[0])
    y = int(aim_y * resolution[1])
    p1 = (x, 0)
    p2 = (x, resolution[1])
    p3 = (0, y)
    p4 = (resolution[0], y)
    cv2.line(frame, p1, p2, (255, 255, 255))
    cv2.line(frame, p3, p4, (255, 255, 255))


def get_z(z):
    return -330.0 * z + 201.0


old_z = z


def down():
    global mover, old_z
    pose = mover._get_current_pose()
    # old/_z, pose[2] = pose[2], -0.12
    mover.move(pose, move=True)


def up():
    global mover, old_z
    pose = mover._get_current_pose()
    pose[2] = old_z
    mover.move(pose, move=True)


def take():
    global grip
    grip.close()


def uptake():
    global grip
    grip.open()


def lookat(frame):
    cframe = frame.copy()

    draw_aim(cframe)

    global mover, x, dc, y, z, lookAt, x_px, y_px, aim_x, aim_y

    key = cv2.waitKey(10) & 0xff
    if key == ord('w'):
        x += dc
        print 'x =', x
        mover.move([x, y, z, -3.14, 0.0, 0.0], move=True)
    elif key == ord('s'):
        x -= dc
        print 'x =', x
        mover.move([x, y, z, -3.14, 0.0, 0.0], move=True)
    elif key == ord('d'):
        y -= dc
        print 'y =', y
        mover.move([x, y, z, -3.14, 0.0, 0.0], move=True)
    elif key == ord('a'):
        y += dc
        print 'y =', y
        mover.move([x, y, z, -3.14, 0.0, 0.0], move=True)
    elif key == ord('t'):
        mover.move([0.5, 0.0, 0.1, -3.14, 0.0, 0.0], move=True)
    elif key == ord('g'):
        down()
    elif key == ord('h'):
        up()
    elif key == ord('j'):
        aim_x -= 0.01
    elif key == ord('l'):
        aim_x += 0.01
    elif key == ord('i'):
        aim_y -= 0.01
    elif key == ord('k'):
        aim_y += 0.01
    elif key == ord('n'):
        take()
    elif key == ord('m'):
        uptake()
    elif key == ord('z'):
        z += dc
        if mover.move([x, y, z, -3.14, 0.0, 0.0], move=True):
            print 'z =', z
        else:
            print 'Not suc z down'
    elif key == ord('x'):
        z -= dc
        if mover.move([x, y, z, -3.14, 0.0, 0.0], move=True):
            print 'z =', z
        else:
            print 'Not suc z down'
    if lookAt:
        h, w = cframe.shape[:2]
        z_px = get_z(z)
        x = 0.157 / z_px * (y_px - h * aim_y)
        y = 0.157 / z_px * (x_px - w * aim_x)
        print x, y, z, z_px
        lookAt = False

        pose = mover._get_current_pose()
        pose[0] += x
        pose[1] += y
        pose[2] = z
        print pose
        mover.move(pose, move=True)
    if cascade:
        pass
        # cubes = cascade.detectMultiScale(cframe)
        # h, w = cframe.shape[:2]
        # if len(cubes) > 0:
        #     print 'Found ', len(cubes), 'rects'
        #
        #     y_px = cubes[0][3] / 2 + cubes[0][1]
        #     x_px = cubes[0][2] / 2 + cubes[0][0]
        #
        #     x = 0.157 / 172 * (y_px - h / 2)
        #     y = 0.157 / 172 * (x_px - w / 2)
        #     print x, y
        #     lookAt = False
        #
        #     pose = mover._get_current_pose()
        #     pose[0] += x
        #     pose[1] += y
        #     print pose
        #     mover.move(pose, move=True)
        # for x, y, w, h in cubes:
        #     p = x, y
        #     rect = x + w, y + h
        #     cv2.rectangle(cframe, p, rect, (255, 255, 255), 2)

    cv2.imshow('Aim', cframe)
    cv2.waitKey(100)
