import numpy as np
from state.state_vector import StateVector14

x = StateVector14(np.array([
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    100,
    200,
    300,
    400,
    500,
    600,
    700,
    800,
    900,
    1000,
    1100,
    1200,
    1300,
    1400
]))

print(x.state)
x.unpack()
print(x.yaw)
x.yaw = 100
x.pack()
print(x.state)