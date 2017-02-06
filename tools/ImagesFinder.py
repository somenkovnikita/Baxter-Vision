import glob
import imghdr
import os


class ImagesFinder:
    """Find all training_set in basedir, making their classes"""
    def __init__(self, basedir):
        # type: (str) -> any
        """
        Images automatic separate to class

        :param basedir: Base directory for search training_set
        """
        self.images_filename = list()
        for elem in os.listdir(basedir):
            check_path = os.path.join(basedir, elem)
            if not os.path.isdir(check_path):
                continue
            pattern = os.path.join(basedir, elem, "*")
            for images_fn in glob.glob(pattern):
                if imghdr.what(images_fn) is None:
                    continue
                temp = (os.path.abspath(images_fn), elem)
                self.images_filename.append(temp)

    def save(self, filename):
        # type: (str) -> None
        """
        :param filename: Set file to save
        """
        with open(filename, "w") as file:
            for image_fn in self.images_filename:
                join_gen = (str(e) for e in image_fn)
                file.write(", ".join(join_gen) + "\n")
