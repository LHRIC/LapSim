from kinematic_model import KinematicModel
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

test_kin_model = KinematicModel()
test_kin_model.from_hardpoints('hardpoints.yaml')
f = test_kin_model.front
r = test_kin_model.rear

ax = plt.figure().add_subplot(projection="3d")
ax.plot_surface(f[:,:,3],f[:,:,4],f[:,:,5])
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_aspect('equal', adjustable='box')
for i in range(np.shape(f)[0]):
    ax.plot(f[i,:,3],f[i,:,4],f[i,:,5],color="black")

plt.show()

bx = plt.figure().add_subplot(projection="3d")
bx.plot(r[0,:,3],r[0,:,4],r[0,:,5])
bx.set_xlabel('x')
bx.set_ylabel('y')
bx.set_zlabel('z')
bx.set_aspect('equal', adjustable='box')

plt.show()

import csv
fil_name = 'data.csv'
data = test_kin_model.front
data = data.tolist()
with open(fil_name, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerows(data)

fil_name = 'data2.csv'
data = test_kin_model.rear
data = data.tolist()
with open(fil_name, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerows(data)
