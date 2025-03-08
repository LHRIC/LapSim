import numpy as np

from vehicle.vehicle import Vehicle
x0 = np.zeros(28)
u0 = np.zeros(3)
car = Vehicle(x0, u0)
x_dot = car.evaluate()
# print(x_dot)
for i, xm in enumerate(x_dot):
    print(i,':', xm)

