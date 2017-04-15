import argparse
import codecs
from os.path import join, basename

import cv2

from collect_images import collect_images
from maps import ClassMap

"""
Script for mark image for generate *.list files

Output format:
<image-path> <class>
"""


def mark_images(basedir):
    results = list()
    mapper = ClassMap('config/class_letter.txt')
    for filename in collect_images(basedir):
        image = cv2.imread(filename)
        resized = cv2.resize(image, (100, 100))
        cv2.imshow('Mark', resized)
        letter = unichr(cv2.waitKey())
        class_ = mapper.get_class(letter)
        result = basename(filename), str(class_)
        results.append(result)
        print result, letter
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
            mark_file.write(' '.join(line) + '\n')
