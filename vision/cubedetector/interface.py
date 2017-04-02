from abc import ABCMeta, abstractmethod

import numpy as np


class ICubeDetector:
    """Interface for cube detect"""

    @abstractmethod
    def cubes(self, image):
        # type: (ICubeDetector, np.array) -> list
        """Find all cubes on image, return list of rectangles"""

    __metaclass__ = ABCMeta
