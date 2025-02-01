import numpy as np
class ControlVector:
    def __init__(self, u0: np.ndarray):
        self.control = u0
    def unpack(self):
        self.steer = self.control[0]
        self.throttle = self.control[1]
        self.brake = self.control[2]
        return