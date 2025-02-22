import numpy as np
from vehicle.vehicle import Vehicle

x0 = np.zeros(28)
x0[3] = np.deg2rad(15)
u0 = np.array([0,0,0])
car = Vehicle(x0, u0)
car.evaluate()