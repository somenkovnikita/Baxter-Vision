import argparse
import os

import cv2


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

    parser.add_argument('-i', '--filepaths', required=True,
                        help='File with paths to images')
    parser.add_argument('-o', '--outpath', required=True,
                        help='Directory for save cutted images')
    args = parser.parse_args()

    with open(args.filepaths) as file_paths:
        paths = (path.strip() for path in file_paths)
        images = map(cv2.imread, paths)

    base = 0
    for image in images:
        cutter = Cutter(image)
        squares = cutter.run()
        scale = cutter.get_scale()
        for i, square in enumerate(squares):
            fn = os.path.join(args.outpath, str(base + i) + '.png')
            s = list(int(round(ss / scale)) for ss in square)
            cv2.imwrite(fn, image[s[1]:s[3], s[0]:s[2]])
        base += len(squares)

