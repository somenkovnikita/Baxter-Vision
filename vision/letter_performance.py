import argparse
import timeit
from os.path import join

import cv2

from letterrecognizer.interface import ILetterRecognizer
from letterrecognizer.template import TemplateLetterRecognizer

# FIXME: issue for generalize performance script

"""
Print performance for letter recognition ways
Test set and train set must be contain file marked.list.
Generate it with tools.mark_images.py
"""


def performance(recognizer, test_set, write_log=True):
    # type: (ILetterRecognizer) -> None
    images = [image for image, _ in test_set]
    corrects = [correct for _, correct in test_set]

    start = timeit.default_timer()
    answers = recognizer.letters(images)
    end = timeit.default_timer()

    results = [ans == cor for ans, cor in zip(answers, corrects)]
    fp = [ans and not cor for ans, cor in zip(answers, corrects)]
    fn = [not ans and cor for ans, cor in zip(answers, corrects)]

    print 'Elapsed time', (end - start) * 1000.0, 'ms'
    print 'Correct recognition:', results.count(True), '/', len(results)
    print 'False Positive:', fp.count(True), '/', len(fp)
    print 'False Negative:', fn.count(True), '/', len(fn)

    if write_log:
        with open('performance.log', 'w') as prf:
            for answer, correct in zip(answers, corrects):
                if answer != correct:
                    templ = 'ans: {0}, cor: {1}\n'
                    line = templ.format(answer, correct)
                    prf.write(line)

    for image, correct in test_set:
        recognizer.letter(image)


def prepare_template_recognition(filename):
    ILetterRecognizer.setup_letters(filename)
    template_recognizer = TemplateLetterRecognizer()
    return template_recognizer


def prepare_test_set(test_set_directory):
    filename = join(test_set_directory, 'marked.list')
    ts = list()
    with open(filename) as test_set_file:
        for line in test_set_file:
            image_fn, letter = line.strip().split()
            image_fn = join(test_set_directory, image_fn)
            image = cv2.imread(image_fn)
            ts.append((image, letter))
    return ts


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Print performance for letter recognition ways')

    parser.add_argument('--test_set_dir', default='../assets/letters/test_set',
                        help='Specify own path to test_set directories')
    parser.add_argument('--training_set_dir', default='../assets/letters/training_set',
                        help='Specify own path to training_set directories')
    parser.add_argument('--method', choices=['template', 'perceptron'],
                        help='Choose method of letter recognition')

    args = parser.parse_args()

    recognizer = None
    if args.method == 'template':
        training_set = join(args.training_set_dir, 'marked.list')
        recognizer = prepare_template_recognition(training_set)
    elif args.method == 'perceptron':
        raise NotImplementedError('For perceptron in future')

    test_set = prepare_test_set(args.test_set_dir)
    performance(recognizer, test_set)
