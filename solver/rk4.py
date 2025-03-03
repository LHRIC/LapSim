import numpy as np
class RK4:
    def __init__(self):
        return
    def solve(fun, x0, t0, h, break_condition):
        def derivative(t,x,fun):
            return np.array([x[1],fun(t,x)])
        x = x0
        t = t0
        abort = break_condition(t,x)
        x_list = []
        t_list = []
        while abort == False:
            k1 = derivative(t,x, fun)
            k2 = derivative(t+h/2, x+h*(k1/2), fun)
            k3 = derivative(t+h/2, x+h*(k2/2), fun)
            k4 = derivative(t+h, x+h*k3, fun)
            t = t + h
            x = x + (h/6)*(k1+2*k2+2*k3+k4)
            x_list.append(x)
            t_list.append(t)
            abort = break_condition(t,x)
        return np.array(t_list), np.array(x_list)