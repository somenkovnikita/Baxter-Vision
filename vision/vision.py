# -*- coding: utf-8 -*-
import cv2

from letterrecognizer.template import ILetterRecognizer
from letterrecognizer.template import TemplateLetterRecognizer

"""
Module for cube detecting and letter recognize
"""

# for draw ru letter as en letter
ru_to_en = {
    'а': 'a', 'б': 'b', 'в': 'v', 'г': 'g',
    'д': 'd', 'е': 'e', 'ё': 'yo', 'ж': 'zh',
    'з': 'z', 'и': 'i', 'й': 'Ji', 'к': 'k',
    'л': 'l', 'м': 'm', 'н': 'n', 'о': 'o',
    'п': 'p', 'р': 'r', 'с': 's', 'т': 't',
    'у': 'u', 'ф': 'f', 'х': 'kh', 'ц': 'ts',
    'ч': 'ch', 'ш': 'sh', 'щ': 'shch', 'ъ': 'tv znak',
    'ь': 'mg znak', 'э': 'iE', 'ю': 'Yu', 'я': 'Ya'
}


cascade = None
letter_recognizer = None


def init(config):
    global cascade, letter_recognizer

    cascade = cv2.CascadeClassifier()
    if not cascade.load(config):
        raise Exception("Cascade not load")

    # FIXME: hard code path!
    ILetterRecognizer.setup_letters('assets/letters/training_set/marked.list')
    letter_recognizer = TemplateLetterRecognizer()


def run(frame):
    global cascade, letter_recognizer

    image = frame.copy()
    cubes = cascade.detectMultiScale(image)

    letters = list()
    for x, y, w, h in cubes:
        start = x, y
        end = x + w, y + h
        color = 255, 255, 255

        cutted_cube = frame[x: x + w, y: y + h]

        if cutted_cube.size:
            letter = letter_recognizer.letter(cutted_cube)
            letter = ru_to_en.get(letter)
            font, scale = cv2.FONT_HERSHEY_SIMPLEX, 1
            cv2.putText(image, letter, start, font, scale, color)
            letters.append(letter)

        cv2.rectangle(image, start, end, color, 2)

    if len(cubes) > 0:
        found = " ".join(map(str, letters))
        print "Found %d rectangle(s)" % len(cubes), "r",  found

    cv2.imshow("Cascade", image)
