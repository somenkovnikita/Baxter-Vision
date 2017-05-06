import argparse
from imghdr import what as isimage
from os import listdir
from os.path import isfile, join, abspath

import cv2


def collect_images(basedir):
    full_path = abspath(basedir)
    all_files = (join(full_path, fn) for fn in listdir(basedir))
    return [fn for fn in all_files if isfile(fn) and isimage(fn)]

class Cutter:
    """Class create window for cut image"""
    win_name = 'cutter'
    exit_keys = {27, ord('q')}
    color_rect = 255, 255, 255

    def __init__(self, image_):
        scale = 1000.0 / image_.shape[1]
        self.scale = scale
        self.image = cv2.resize(image_, dsize=None, fx=scale, fy=scale)
        self.drawing = False
        self.squares = list()
        cv2.namedWindow(Cutter.win_name)
        cv2.setMouseCallback(Cutter.win_name, self.draw_squares)
        self.update()

    def draw_squares(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            square = x, y, x, y
            self.squares.append(square)
            self.drawing = True

        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            sx, sy, ex, ey = self.squares[-1]
            ds = max(x - sx, y - sy)
            self.squares[-1] = sx, sy, sx + ds, sy + ds

        elif event == cv2.EVENT_LBUTTONUP and self.drawing:
            self.drawing = False

        self.update()

    def update(self):
        copy_image = self.image.copy()
        color = Cutter.color_rect
        for x1, y1, x2, y2 in self.squares:
            p1, p2 = (x1, y1), (x2, y2)
            cv2.rectangle(copy_image, p1, p2, color)
        cv2.imshow(Cutter.win_name, copy_image)
            
    def run(self):
        while True:
            key = cv2.waitKey(20) & 0xFF
            if key in Cutter.exit_keys:
                break

            if key == ord('z') and self.squares:
                self.squares.pop()
                self.update()
        return self.squares

    def get_scale(self):
        return self.scale


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Cut images into square for create dataset')

    parser.add_argument('-i', '--input_dir', required=True,
                        help='File with paths to images')
    parser.add_argument('-o', '--output_dir', required=True,
                        help='Directory for save cutted images')
    args = parser.parse_args()

    paths = collect_images(args.input_dir)
    images = map(cv2.imread, paths)

    base = 0
    for image in images:
        cutter = Cutter(image)
        squares = cutter.run()
        scale = cutter.get_scale()
        for i, square in enumerate(squares):
            fn = join(args.output_dir, str(base + i) + '.png')
            s = list(int(round(ss / scale)) for ss in square)
            cv2.imwrite(fn, image[s[1]:s[3], s[0]:s[2]])
        base += len(squares)


class ClickChecker:
    def __init__(self, win_name):
        self.clicks = list()
        cv2.setMouseCallback(win_name, self.clicker_callback)

    def clicker_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicks.append((x, y))

    def get_clicks(self):
        ref, self.clicks = self.clicks, list()
        return ref
