# -*- coding: utf-8 -*-
import cv2

from tools.image_cutter import ClickChecker
from tools.maps import ClassMap
from vision.letterrecognizers import SVMLetterRecognizer

"""
Module for cube detecting and letter recognize
"""

# for draw ru letter as en letter
ru_to_en = {
    u'а': 'A', u'б': 'b', u'в': 'v', u'г': 'g',
    u'д': 'd', u'е': 'e', u'ё': 'yo', u'ж': 'zh',
    u'з': 'z', u'и': 'i', u'й': 'Ji', u'к': 'k',
    u'л': 'l', u'м': 'm', u'н': 'n', u'о': 'o',
    u'п': 'p', u'р': 'r', u'с': 's', u'т': 't',
    u'у': 'u', u'ф': 'f', u'х': 'kh', u'ц': 'ts',
    u'ч': 'ch', u'ш': 'sh', u'щ': 'shch', u'ъ': 'tv znak',
    u'ь': 'mg znak', u'э': 'iE', u'ю': 'Yu', u'я': 'Ya'
}

classmap = ClassMap('config/class_letter.txt')
cascade = None
letter_recognizer = None
clicks = None


def init(config):
    global cascade, letter_recognizer, clicks

    cascade = cv2.CascadeClassifier()
    if not cascade.load(config):
        raise Exception("Cascade not load")

    # FIXME: hard code path!
    # ILetterRecognizer.setup_letters('assets/letters/training_set/marked.list')
    letter_recognizer = SVMLetterRecognizer('config/letter_recognizer.svm')

    cv2.namedWindow('Cascade')
    clicks = ClickChecker('Cascade')

i = 0


def run(frame):
    global cascade, letter_recognizer,i, clicks

    image = frame.copy()
    image = cv2.flip(image, 1)
    cubes = cascade.detectMultiScale(image)

    cutted_cubes = list()
    cubes_coordinates = list()
    color = 255, 255, 0
    for x, y, w, h in cubes:
        start = x, y
        end = x + w, y + h

        cutted_cube = image[y: y + h, x: x + w]

        if cutted_cube.size:
            cutted_cubes.append(cutted_cube)
            cubes_coordinates.append((x, y, x + w, y + h))

        cv2.rectangle(image, start, end, color, 2)

    letters = list()
    if cutted_cubes:
        letters = letter_recognizer.letters(cutted_cubes)

    for i, cutted_cube in enumerate(cubes_coordinates):
        start = tuple(cutted_cube[:2])
        class_ = letters[i]
        letter = classmap.get_letter(class_)
        letter = ru_to_en.get(letter)
        font, scale = cv2.FONT_HERSHEY_SIMPLEX, 1
        try:
            cv2.putText(image, letter, start, font, scale, color)
        except Exception as e:
            print e, letter, start, font, scale, color
            raise e

    if len(cubes) > 0:
        found = " ".join(map(str, letters))
        print "Found %d rectangle(s)" % len(cubes), found

    cv2.imshow("Cascade", image)
