import numpy as np
from numpy.typing import _80Bit
import scipy.optimize as opt

#dist, speed
low_data = [
[0.60, 13000],
[0.90, 14037],
[1.22, 15950],
[1.55, 17752],
[1.84, 19817],
[2.18, 22500],
[2.53, 24358]
]

'''high_data = [
[1.53, 18578],
[1.88, 20642],
[2.15, 22294],
[2.49, 24358],
[2.70, 26009],
[3.02, 27248],
[3.40, 28899],
[3.68, 30550],
]'''

high_data = [
[1.53, 18578],
[1.88, 20642],
[2.15, 22294],
[2.49, 24358],
[2.70, 26009],
[3.02, 27248],
[3.40, 28899],
[3.68, 30550],
]

class ThrowerTraining():
    def __init__(self):
        self.low_a, self.low_b = self.fit_model(low_data)
        self.high_a, self.high_b = self.fit_model(high_data)

        self.angle_dist_thresholds = [1.8, 2.2] #[low, high] in meters

        self.thrower_high = 33
        self.thrower_low = 0

        self.thrower_angle = self.thrower_low

    def speed_function(self, x, a, b):
        return a*x+b

    def fit_model(self, data):
        speed_data = [x[1] for x in data]
        dist_data = [x[0] for x in data]
        optimized_parameters, _ = opt.curve_fit(
            self.speed_function,
            dist_data,
            speed_data,
            bounds=([-np.inf, -np.inf], [np.inf, np.inf])
            )

        return optimized_parameters

    def get_speed(self, dist):
        if dist <= self.angle_dist_thresholds[0]:
            self.thrower_angle = self.thrower_low
        elif dist >= self.angle_dist_thresholds[1]:
            self.thrower_angle = self.thrower_high

        if self.thrower_angle == self.thrower_low:
            speed = self.speed_function(dist, self.low_a, self.low_b)
        elif self.thrower_angle == self.thrower_high:
            speed = 0.9*self.speed_function(dist, self.high_a, self.high_b)
        else:
            print("Unknown thrower angle state")
            speed = 0

        if speed < 0:
            return 0
        if speed > 45000:
            return 45000

        return int(speed), int(self.thrower_angle)
