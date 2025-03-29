import numpy as np
from vehicle.vehicle import Vehicle
from state.state_vector import StateVector14

dummy_state = StateVector14()
names_vec = dummy_state.names_vec + dummy_state.names_vec_dt
names_dt_vec = dummy_state.names_vec_dt + dummy_state.names_vec_ddt
x0 = np.zeros(28)
x0[2] = 1.82510552
x0[6:10] = [1.59160527,   1.59160527, -20.99110329, -20.99110329]

u0 = np.zeros(3)
car = Vehicle(x0, u0)
x_dot = car.evaluate()
# print(x_dot)
for i, xm in enumerate(x_dot):
    print(names_dt_vec[i],':', xm)

