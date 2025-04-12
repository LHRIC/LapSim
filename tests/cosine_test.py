import numpy as np
import matplotlib.pyplot as plt
from solver.rk4 import RK4

def cosine_test():
    def fun(t, x):
        k = 1
        b = 0.5
        return [x[1],-k*x[0] - b*x[1]]

    def check(t, x):
        if t > 20:
            zero = True
        else:
            zero = False
        return zero

    t_list, x_list = RK4.solve(fun=fun, x0=[1,0], t0=0, h=0.01, break_condition=check)
    plt.plot(t_list,x_list[:,0])
    # plt.plot(t_list,np.cos(t_list),linestyle="dotted",color="black")
    plt.show()
    return