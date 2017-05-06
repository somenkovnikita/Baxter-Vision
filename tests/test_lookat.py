# coding=utf-8
import baxter_interface
import cv2

from baxter.hand import HandMover
from baxter.translators import CoordinatesTranslator

TEST_NAME = "Test Baxter cubes search(look at)"

# Центр в % от начала
aim_x = 0.56
aim_y = 0.45
# Приращение для перемещения по координатам
dc = 0.03
dmove = 0.157
x = 0.5
y = 0.0
# 0.1 = 32см
z = 0.1
lookAt = False
isGripped = False
x_px = 0
y_px = 0
old_z = z
translator = None
mover = None
cascade = None
grip = None


# Обработка действий мыши
# noinspection PyUnusedLocal
def mouse_callback(event, x, y, flags, param):
    global lookAt, x_px, y_px
    if event == cv2.EVENT_LBUTTONDOWN:
        lookAt = (x, y)
        x_px, y_px = lookAt
        print x, y
    if event == cv2.EVENT_RBUTTONDOWN:
        print x, y


# Инициализация классов теста
def init(config):
    global mover, x, y, cascade, grip, translator
    cv2.namedWindow(TEST_NAME)
    cv2.setMouseCallback(TEST_NAME, mouse_callback)
    grip = baxter_interface.Gripper("left")
    grip.reset_custom_state()
    cascade = cv2.CascadeClassifier()
    translator = CoordinatesTranslator((aim_x, aim_y), dmove)
    mover = HandMover.HandMover("left")

    if cascade.load(config):
        print "Load cascade success"


# Отрисовка перекрестья для позиционирования
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


# Реальная высота до объекта
# Для координат робота
def get_rz(robot_z):
    return -330.0 * robot_z + 201.0


# Методы перемещения и захвата
def down():
    global mover, old_z
    pose = mover.get_current_pose()
    mover._move(pose, move=True)


def up():
    global mover, old_z
    pose = mover.get_current_pose()
    pose[2] = old_z
    mover._move(pose, move=True)


def take():
    if isGripped:
        global grip
        grip.open()
    else:
        global grip
        grip.close()


# Основной цикл
def look_at(frame):
    cframe = frame.copy()

    draw_aim(cframe)

    global mover, x, dc, y, z, lookAt, x_px, y_px, aim_x, aim_y

    # WASD управление перемещением манипулятора
    key = cv2.waitKey(10) & 0xFF
    # W
    if key == ord("w"):
        x += dc
        print "x =", x
        mover._move([x, y, z, -3.14, 0.0, 0.0], move=True)
    # A
    elif key == ord("a"):
        y += dc
        print "y =", y
        mover._move([x, y, z, -3.14, 0.0, 0.0], move=True)
    # S
    elif key == ord("s"):
        x -= dc
        print "x =", x
        mover._move([x, y, z, -3.14, 0.0, 0.0], move=True)
    # D
    elif key == ord("d"):
        y -= dc
        print "y =", y
        mover._move([x, y, z, -3.14, 0.0, 0.0], move=True)
    # Вверх\Вниз
    elif key == ord("q"):
        down()
    elif key == ord("e"):
        up()
    # Захват\Выкл захват
    elif key == ord(" "):
        take()
    # Движение вверх\вниз по z
    elif key == ord("z"):
        z += dc
        if mover._move([x, y, z, -3.14, 0.0, 0.0], move=True):
            print "z =", z
        else:
            print "Error z down"
    elif key == ord("x"):
        z -= dc
        if mover._move([x, y, z, -3.14, 0.0, 0.0], move=True):
            print "x =", z
        else:
            print "Error x down"
    # Установка робота на позицию
    if lookAt:
        z_px = get_rz(z)
        translator.set_resolution(cframe.shape[:2])
        x, y = translator.translate(x_px, y_px, z_px)
        print x, y, z, z_px
        lookAt = False

        pose = mover.get_current_pose()
        pose[0] += x
        pose[1] += y
        pose[2] = z
        print pose
        mover._move(pose, move=True)
    if cascade:
        pass
    cv2.imshow(TEST_NAME, cframe)
    cv2.waitKey(100)
