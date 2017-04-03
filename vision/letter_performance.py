import timeit

from letterrecognizer.interface import ILetterRecognizer
from letterrecognizer.template import TemplateLetterRecognizer

# FIXME: issue for generalize performance script

"""Print performance for letter recognition ways"""


def performance(recognizer, test_set):
    # type: (ILetterRecognizer) -> None
    images = [image for image, _ in test_set]
    corrects = [correct for _, correct in test_set]

    start = timeit.default_timer()
    answers = recognizer.letters(images)
    end = timeit.default_timer()

    results = [ans == cor for ans, cor in
               zip(answers, corrects)]
    fp = [ans is True and cor is False
          for ans, cor in zip(answers, corrects)]
    fn = [ans is False and cor is True
          for ans, cor in zip(answers, corrects)]

    print 'Elapsed time', end - start, 'ms'
    print 'Correct recognition:', results.count(True), '/', len(results)
    print 'False Positive:', fp.count(True), '/', len(fp)
    print 'False Negative:', fn.count(True), '/', len(fn)

    for image, correct in test_set:
        recognizer.letter(image)


def prepare_template_recognition(filename):
    template_recognizer = TemplateLetterRecognizer()
    template_recognizer.setup_letters(filename)
    return template_recognizer

if __name__ == '__main__':

    pass
