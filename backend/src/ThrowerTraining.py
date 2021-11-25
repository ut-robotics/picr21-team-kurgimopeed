import numpy as np
from numpy.typing import _80Bit
import scipy.optimize as opt

data = [
[1.14, 758],
[1.18, 733],
[1.34, 798],
[1.37, 826],
[1.42, 843],
[1.47, 851],
[1.53, 918],
[1.69, 1000],
[1.75, 1145],
[1.8, 1041],
[1.9, 1080],
[2.03, 1250],
[2.31, 1355],
[2.72, 1590],
]

class ThrowerTraining():
    def __init__(self):
        self.speed_data = [x[1] for x in data]
        self.dist_data = [x[0] for x in data]

        self.a, self.b = self.fit_model()

    def speed_function(self, x, a, b):
        return a*x+b

    def fit_model(self):
        optimized_parameters, _ = opt.curve_fit(
            self.speed_function,
            self.dist_data,
            self.speed_data,
            bounds=([-np.inf, -np.inf], [np.inf, np.inf])
            )

        return optimized_parameters

    def get_speed(self, dist):
        speed = self.speed_function(dist, self.a, self.b)
        if speed < 0:
            return 0
        if speed > 2500:
            return 2500
        return int(speed)
