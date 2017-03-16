import argparse
import glob
import imghdr
import os

import cv2

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--in_dir", type=str, default=".",
                    help="Input dir for images")

parser.add_argument("-o", "--out_dir", type=str, default=None,
                    help="Output dir for images")

parser.add_argument("-s", "--resize", type=str, default=None)

args = parser.parse_args()
in_base = os.path.join(args.in_dir, "*")
if args.out_dir is None:
    args.out_dir = args.in_dir
count = 0
files = glob.glob(in_base)
print args.out_dir
for fn in files:
    if imghdr.what(fn) is not None:
        img = cv2.imread(fn)
        os.remove(fn)
        while True:
            img_name = os.path.join(args.out_dir, str(count) + ".jpg")
            if not os.path.exists(img_name):
                break
            count += 1
        shp = img.shape
        rh, rw = int(shp[0] / 2), int(shp[1] / 2)
        sm = min(rh, rw)
        img = img[rh-sm:rh+sm, rw-sm:rw+sm]
        rsz_img = img
        if args.resize is not None:
            w = int(args.resize)
            rsz_img = cv2.resize(img, dsize=(w, w))
        cv2.imwrite(img_name, rsz_img)

