import argparse
import codecs
from glob import glob
from imghdr import what
from os.path import join, basename

import cv2

"""Script for mark image for generate *.list files"""


def mark_images(base_dir):
    results = list()
    for filename in glob(join(base_dir, '*')):
        if what(filename) is None:
            continue
        image = cv2.imread(filename)
        resized = cv2.resize(image, (100, 100))
        cv2.imshow('Mark', resized)
        key = cv2.waitKey()
        result = basename(filename), unichr(key)
        results.append(result)
        print result
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Script for mark image for generate *.list files')

    parser.add_argument('-i', '--input_dir', help='Input dir for work', default='.')
    parser.add_argument('-o', '--output_file', help='Output file path *.list')

    args = parser.parse_args()
    if args.output_file is None:
        args.output_file = join(args.input_dir, 'marked.list')

    makred = mark_images(args.input_dir)
    with codecs.open(args.output_file, 'w', encoding='utf-8') as mark_file:
        for line in makred:
            mark_file.write('\t'.join(line) + '\n')
