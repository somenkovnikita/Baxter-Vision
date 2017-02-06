import cv2


class NeuralNetworkParameters:
    criteria_types = {
        cv2.TERM_CRITERIA_COUNT,
        cv2.TERM_CRITERIA_EPS,
        cv2.TERM_CRITERIA_MAX_ITER
    }
    train_methods = {
        cv2.ANN_MLP_TRAIN_PARAMS_BACKPROP,
        cv2.ANN_MLP_TRAIN_PARAMS_RPROP,
    }
    default_max_steps = 10000
    default_max_error = 0.0001
    default_stop_criteria = (
        cv2.TERM_CRITERIA_COUNT |
        cv2.TERM_CRITERIA_EPS,
        default_max_steps,
        default_max_error)
    default_parameters = dict(
        term_crit=default_stop_criteria,
        train_method=cv2.ANN_MLP_TRAIN_PARAMS_RPROP,
        bp_dw_scale=0.01,
        bp_moment_scale=0.1,
        rp_dw0=0.1,
        rp_dw_plus=1.2,
        rp_dw_minus=0.5,
        rp_dw_min=1.19209e-07,
        rp_dw_max=50.0
    )

    def __init__(self, **kwargs):
        self.parameters = self.default_parameters
        self.set(**kwargs)

    def set(self, **kwargs):
        self.parameters.update(kwargs)

    def get(self):
        return self.parameters
