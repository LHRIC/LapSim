from kinematics.kinematic_model import KinematicModel
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.ticker as FuncFormatter
import numpy as np

test_kin_model = KinematicModel()
test_kin_model.from_hardpoints('parameters/hardpoints.yaml')
f = test_kin_model.front
r = test_kin_model.rear

fig = plt.figure(figsize=(16,6))
fig.suptitle('Front Camber Gain')
fig.tight_layout()

n = len(f[0,:,0])
colormap = plt.cm.managua
colors = [colormap(i) for i in np.linspace(1, 0, n)]
norm = mpl.colors.Normalize(vmin=-1,vmax=1)
scalar_mappable = plt.cm.ScalarMappable(cmap=plt.cm.managua,norm=norm)

ax1 = fig.add_subplot(121)

for i in range(n):
    ax1.plot(f[:,i,0], np.rad2deg(f[:,i,9]))

for i, line in enumerate(ax1.lines):
    line.set_color(colors[i])

ax1.set_xlabel('Shock Compression/Rebound (mm)')
ax1.set_ylabel('Angle (deg)')
ax1.set_title('Camber Gain')

n = len(f[:,0,0])
colormap = plt.cm.managua
colors = [colormap(i) for i in np.linspace(1, 0, n)]
norm = mpl.colors.Normalize(vmin=-1,vmax=1)
scalar_mappable = plt.cm.ScalarMappable(cmap=plt.cm.managua,norm=norm)

ax2 = fig.add_subplot(122)
for i in range(n):
    ax2.plot(f[i,:,1], np.rad2deg(f[i,:,9]))

for i, line in enumerate(ax2.lines):
    line.set_color(colors[i])

ax2.set_xlabel('Steering Rack Displacement (mm)')
ax2.set_ylabel('Angle (deg)')
ax2.set_title('Steer Camber')

cbar = plt.colorbar(scalar_mappable,ax=ax1,label='Fractional Steer')
cbar = plt.colorbar(scalar_mappable,ax=ax2,label='Fractional Bump')

plt.show()