from kinematics.parameter_model import KinematicModel
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

test_kin_model = KinematicModel()
test_kin_model.from_parameters('parameters.yaml')
f = test_kin_model.front
r = test_kin_model.rear

fig = plt.figure()
fig.suptitle('Contact Patch Configuration Spaces')
fig.subplots_adjust(left=0.1, bottom=None, right=0.9, top=None, wspace=0.1, hspace=None)
ax1 = fig.add_subplot(121,projection="3d")
ax1.plot_surface(f[:,:,3],f[:,:,4],f[:,:,5])
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
ax1.set_title('Front')

ax2 = fig.add_subplot(122,projection="3d")
ax2.plot(r[:,0,3],r[:,0,4],r[:,0,5])
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z')
ax2.set_title('Rear')

plt.show()

import csv
fil_name = 'front_kin_surrogate.csv'
data = test_kin_model.front
data = data.tolist()
with open(fil_name, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerows(data)

fil_name = 'rear_kin_surrogate.csv'
data = test_kin_model.rear
data = data.tolist()
with open(fil_name, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerows(data)
