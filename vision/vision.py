import cv2

from letterrecognizer.template import ILetterRecognizer
from letterrecognizer.template import TemplateLetterRecognizer

"""
Module for cube detecting and letter recognize
"""

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

    if len(cubes) > 0:
        print "Found ", len(cubes), "rectangle(s)"

    for x, y, w, h in cubes:
        point = x, y
        rect = x + w, y + h
        color = (255, 255, 255)

        cutted_cube = frame[x: x + w, y: y + h]

        if cutted_cube.size:
            letter = letter_recognizer.letter(cutted_cube)
            cv2.putText(image, letter, point,
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color)

        cv2.rectangle(image, point, rect, color, 2)

    cv2.imshow("Cascade", image)
