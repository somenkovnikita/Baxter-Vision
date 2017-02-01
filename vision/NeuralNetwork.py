import numpy as np
import cv2


class NeuralNetwork:
    def __init__(self, layers, step_size=0.01, steps=10000, max_err=0.0001, momentum=0.0):
        # TODO: add doc-string
        self.layers = np.array(layers)
        self.network = cv2.ANN_MLP(self.layers)
        criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, steps, max_err)
        self.params = {
            "term_crit": criteria,
            "train_method": cv2.ANN_MLP_TRAIN_PARAMS_RPROP,
            "bp_dw_scale": step_size,
            "bp_moment_scale": momentum
        }

    def train(self, input_set, output_set, log=None):
        # TODO: add doc-string
        # TODO: add logging of training time
        if log:
            print "Training..."
        sample_weights = None
        iteration = self.network.train(
            input_set, output_set, sample_weights,
            params=self.params)
        if log:
            print "Trained with %d iterations" % iteration
        return iteration

    def classify(self, input_samples):
        # type: (np.array) -> (int, np.array)
        ret, outputs = self.network.predict(input_samples)
        return ret, outputs
