import codecs

class ClassMap:
    """Contain info letter <-> class"""
    def __init__(self, model):
        # type: (ClassMap, str) -> None
        with codecs.open(model, encoding='utf-8') as model_file:
            matches = map(unicode.split, model_file)
        self.ltr2cls = {l: int(c) for l, c in matches}
        self.cls2ltr = {int(c): l for l, c in matches}

    def get_letter(self, class_):
        return self.cls2ltr[class_]

    def get_class(self, letter_):
        return self.ltr2cls[letter_]
