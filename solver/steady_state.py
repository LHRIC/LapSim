# pyright: ignore
# type: ignore

import numpy as np
from scipy.optimize import root 
from scipy.optimize import least_squares
from scipy.optimize import basinhopping
from vehicle.vehicle import Vehicle

class SteadyStateSolver:
    
    def __init__(self, x_map, x_dot_1, x0, u0):
        """
            Initialzes Steady State Solver Object.

            Parameters:
                x_map: np.ndarray of True or False corresponding to variables to implicitly\
                    solve for in the state vector x
                x_dot_1: np.ndarray of float or None corresponding to desired values of d/dt x\
                    (# of floats in x_dot_1 should = # of True in x_map)
                x0: np.ndarray of float, initial state vector
                u0: np.ndarray of float, initial control vector
        """
        self.car = Vehicle(x0, u0)
        self.x_map = x_map
        self.x_dot_1 = x_dot_1
        self.tracker = []
        return
    
    def solve_implicit(self, x0):
        i, j = 0, 0
        x0_mod = []
        for xm in self.x_map:
            if xm == True:
                x0_mod.append(x0[i])
            i += 1
        x0_mod = np.array(x0_mod)
        sol = root(self._fun_implicit, x0_mod, method='df-sane', options={'maxfev': 20000, 'xtol': 1e-12}) 
        # sol = basinhopping(self._fun_implicit_scalar, x0_mod, niter=100)
        return sol
    
    def _fun_implicit(self, x):
        i, j = 0, 0
        for xm in self.x_map:
            if xm == True:
                self.car.state_vector.state[i] = x[j]
                j += 1
            i += 1
        x_dot = self.car.evaluate()
        self.tracker.append([np.copy(self.car.state_vector.state),np.copy(x_dot)])
        i, j = 0, 0
        arg = np.zeros(len(x))
        for xm in self.x_dot_1:
            if xm is not None:
                arg[j] = x_dot[i] - self.x_dot_1[i]
                j += 1
            i += 1
        return arg
    
    def _fun_implicit_scalar(self, x):
        i, j = 0, 0
        for xm in self.x_map:
            if xm == True:
                self.car.state_vector.state[i] = x[j]
                j += 1
            i += 1
        x_dot = self.car.evaluate()
        self.tracker.append([np.copy(self.car.state_vector.state),np.copy(x_dot)])
        i, j = 0, 0
        arg = np.zeros(len(x))
        for xm in self.x_dot_1:
            if xm is not None:
                arg[j] = x_dot[i] - self.x_dot_1[i]
                j += 1
            i += 1
        return np.sum(arg**2)