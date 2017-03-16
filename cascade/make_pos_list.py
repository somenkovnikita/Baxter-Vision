import glob
import os
import sys

import cv2

if len(sys.argv) < 1:
    print "Required search path!"

aim = os.path.join(sys.argv[1], "*.png")
pattern = "{0} {1} {2}"
with open("pos.txt", "w") as pos:
    for fn in glob.glob(aim):
        img = cv2.imread(fn)
        s = img.shape
        to_write = (fn,
                1, 0, 0, s[0], s[1])
        pos.write(" ".join(str(i) for i in to_write) + "\n")
