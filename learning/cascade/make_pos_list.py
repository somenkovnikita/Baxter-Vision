import glob
import os
import sys

import cv2

"""Create positive list for cascade train"""

if len(sys.argv) < 1:
    print "Required search path!"

aim = os.path.join(sys.argv[1], "*")
pattern = "{0} {1} {2}"

for fn in glob.glob(aim):
    s = cv2.imread(fn).shape
    to_write = fn, 1, 0, 0, s[0], s[1]
    print " ".join(map(str, to_write))
