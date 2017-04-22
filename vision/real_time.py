import argparse
import importlib

import cv2

"""
Subsystem for run yours module for work with camera
"""


def parse_baxter_camera(camera_name):
    _, cam = camera_name.split("_")
    return cam + "_hand" if cam != "head" else cam


def get_camera(camera_name):
    if "baxter" in camera_name:
        from rospy import init_node
        from baxter import Camera
        init_node("real_time_camera")
        return Camera.Camera(parse_baxter_camera(camera_name))
    else:
        from tools.localcamera import LocalCamera
        return LocalCamera()


def simple_show(frame):
    cv2.imshow("Live", frame)


def run(cam, callback):
    while True:
        cam.read_frame()
        frame = cam.get_frame()

        callback(frame)

        key = cv2.waitKey(30) & 0xff
        if key == ord("q"):
            break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Apply function for real time camera")
    parser.add_argument("-c", "--camera", help="Camera type", default="local",
                        choices=["local", "baxter_left", "baxter_right", "baxter_head"])
    parser.add_argument("-m", "--module", help="Module name for search function", default=None)
    parser.add_argument("-f", "--func", help="Function(callback) to run", default=simple_show)
    parser.add_argument("-C", "--config", help="Config file for run module", default=None)

    args = parser.parse_args()
    camera = get_camera(args.camera)
    aim_func = args.func
    if args.module is not None:
        aim_module = importlib.import_module(args.module)
        aim_func = getattr(aim_module, args.func)
        aim_module.init(args.config)

    run(camera, aim_func)
