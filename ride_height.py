import numpy as np
from solver.steady_state import SteadyStateSolver
dim = 28
x_map = np.full(dim, False)
x_dot_1 = np.full(dim, None)
x0 = np.zeros(dim)
u0 = np.zeros(3)

x_map[0:10] = True
x_map[6:10] = True
x_dot_1[14:20] = 0
x_dot_1[20:24] = 0

solver = SteadyStateSolver(x_map, x_dot_1, x0, u0)
sol = solver.solve_implicit(x0)
print(sol.x)
print(sol.fun)
print(sol.message)

