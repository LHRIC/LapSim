import numpy as np
import matplotlib.pyplot as plt
from vehicle.vehicle import Vehicle
from state.state_vector import StateVector14
from solver.rk4 import RK4
from scipy.integrate import solve_ivp
import time
import cProfile

dummy_state = StateVector14()
names_vec = dummy_state.names_vec + dummy_state.names_vec_dt
names_dt_vec = dummy_state.names_vec_dt + dummy_state.names_vec_ddt
x0 = np.zeros(28)

line_styles = [
    'r-', 'g--', 'b-.', 'k:', 
    'c-', 'm--', 'y-.', 'r:', 
    'g-', 'b--', 'k-.', 'c:', 
    'm-', 'y--', 'r-.', 'g:', 
    'b-', 'k--', 'c-.', 'm:', 
    'y-', 'r--', 'g-.', 'b:', 
    'k-', 'c--', 'm-.', 'y:'
]

# x0[2] = 1.82510552
x0[2] = 20
u0 = np.zeros(3)
car = Vehicle(x0, u0)
t0 = 0
h = 0.0005
tmax = 2

def break_fun(t,x): 
    if t>=tmax:
        print('Done!')
        return True
    else:
        return False

def fun(t,x):
    car.state_vector.state = x
    x_dot = car.evaluate()
    if round(t*10)%1 == 0:
        print(f'{t}' ,end='\r')
        pass
    return x_dot

# profiler = cProfile.Profile()
# profiler.enable()
sol = solve_ivp(fun,(0,1.5),x0, method='RK45') #RK45 or BDF
# profiler.disable()
# profiler.print_stats(sort='time')

x_list = np.transpose(sol.y)
t_list = sol.t

# print('',end='\n')
print(len(x_list[:,0]))
# print(x_list[:,20])

# t_list, x_list, fx_list = RK4.solve(fun = fun, x0 = x0, t0 = t0, h=h, break_condition=break_fun)
order_mag = lambda x: int(np.log10(max(np.abs(x_list[:,1,x])) + 1))
for i in range(len(x_list[0])):
    plt.plot(t_list, x_list[:,i]/np.max(np.abs(x_list[:,i] + 1)), line_styles[i], label=names_vec[i])
plt.legend()
plt.title('All States')
plt.show()

plt.plot(t_list,x_list[:,6:8])
plt.title('Front Jounces')
plt.show()

plt.plot(t_list,x_list[:,8:10])
plt.title('Rear Jounces')
plt.show()

plt.plot(t_list,np.rad2deg(x_list[:,4]))
plt.title('Pitch (deg)')
plt.show()

plt.plot(t_list,(x_list[:,2]))
plt.title('Z')
plt.show()

plt.plot(t_list,x_list[:,20:22])
plt.title('Front Jounces dt')
plt.show()

plt.plot(t_list,x_list[:,22:24])
plt.title('Rear Jounces dt')
plt.show()