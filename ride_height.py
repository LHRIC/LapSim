import numpy as np
from solver.steady_state import SteadyStateSolver
import matplotlib.pyplot as plt
from state.state_vector import StateVector14

dim = 28
x_map = np.full(dim, False)
x_dot_1 = np.full(dim, None)
x0 = np.zeros(dim)
u0 = np.zeros(3)

dummy_state = StateVector14()
names_vec = dummy_state.names_vec + dummy_state.names_vec_dt
names_dt_vec = dummy_state.names_vec_dt + dummy_state.names_vec_ddt

x_map[2] = True # z
# x_map[3] = True # roll
x_map[4] = True # pitch
x_map[6:10] = True # z_xx
x_dot_1[16] = 0 # z_ddt
# x_dot_1[17] = 0 # roll_ddt
x_dot_1[18] = 0 # pitch_ddt
x_dot_1[20:24] = 0 # z_xx_ddt

solver = SteadyStateSolver(x_map, x_dot_1, x0, u0)
attempts = 0
converged = False
while converged == False:
    sol = solver.solve_implicit(x0)
    attempts +=1
    # print(f'Success: {sol.success}, {sol.message[:-1]} with {sol.nfev} function evaluations and qtf = {sol.qtf}')
    print(attempts)
    if sol.success == False:
        i, j = 0, 0
        x0 = np.zeros(dim)
        for flag in x_map:
            if flag:
                x0[i] = sol.x[j]
                j+=1
            i+=1
    else:
        converged = True
    if attempts >= 1:
        converged = True
print(f'Success: {sol.success}, {sol.message[:-1]} with {sol.nfev} function evaluations')

i, j = 0, 0
for flag in x_map:
    if flag:
        print(f'{names_vec[i]} = {sol.x[j]}')
        j+=1
    i+=1

i, j = 0, 0
for val in x_dot_1:
    if val is not None:
        print(f'{names_dt_vec[i]} = {sol.fun[j]}')
        j+=1
    i+=1

print(sol.x)

fig, axs = plt.subplots(2)

tracker = np.array(solver.tracker)
iter_n = range(len(tracker[:,0,0]))
order_mag = lambda x: int(np.log10(max(np.abs(tracker[:,1,x])) + 1))
for i, flag in enumerate(x_map):
    if flag:
        axs[0].plot(iter_n, tracker[:,0,i]/(10**order_mag(i)), label=names_vec[i] + f' e {order_mag(i)}')
    if x_dot_1[i] is not None:
        axs[1].plot(iter_n, tracker[:,1,i]/(10**order_mag(i)), label=names_dt_vec[i] + f' e {order_mag(i)}')
axs[0].legend(loc='right')
axs[1].legend(loc='right')
plt.show()
np.save('aisdias',tracker)


