def abstractmethod(method):
    def not_impl_printer(*args, **kwargs):
        print 'This method not impl', method
    not_impl_printer.__doc__ = method.__doc__
    return not_impl_printer


class BaseNeuralNetwork:
    @abstractmethod
    def train(self, images, classes):
        """Train neural, images is list of grayscale images, classes is list of class(any id)"""
        # type: (BaseNeuralNetwork, list, list) -> None
        pass

    @abstractmethod
    def predict(self, image):
        """Get class of input image"""
        pass

    @abstractmethod
    def predict_images(self, images):
        """Get classes of input images"""
        pass