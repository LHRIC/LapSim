from kinematics.kinematic_model import KinematicModel
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.ticker as FuncFormatter
import numpy as np

test_kin_model = KinematicModel()
# test_kin_model.from_hardpoints('parameters/hardpoints.yaml')
test_kin_model.from_xlsx('parameters/HDPT_Export.xlsx', [39.5, 51], [-30, 30, 51], [-30, 30, 51])
f = test_kin_model.front
r = test_kin_model.rear

fig = plt.figure(figsize=(16,6))
fig.suptitle('Contact Patch Configuration Spaces')
fig.tight_layout()
ax1 = fig.add_subplot(121,projection="3d")
ax1.plot_surface(f[:,:,3],f[:,:,4],f[:,:,5])
ax1.quiver(f[:,:,3],f[:,:,4],f[:,:,5], f[:,:,6],f[:,:,7],f[:,:,8], length=1, color='r')
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