import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from state_models.mf_52 import MF52

def run(s,a,fx0,fy0,Cs,Ca):

    fxy = fx0*fy0/np.sqrt(s**2*fy0**2+fx0**2*(np.tan(a))**2)
    fx_1 =  np.sqrt(s**2*Ca**2+(1-s)**2*(np.cos(a))**2*a*fx0**2)/Ca
    fy_1 =  np.sqrt((1-s)**2*(np.cos(a))**2*a*fy0**2+(np.sin(a))**2*a*Cs**2)/(Cs*np.cos(a))

    # fx = (fx0*fy0/np.sqrt(s**2*fy0**2+fx0**2*(np.tan(a))**2))*(np.sqrt(s**2*Ca**2+(1-s)**2*(np.cos(a))**2*a*fx0**2)/Ca)
    # fy = (fx0*fy0/np.sqrt(s**2*fy0**2+fx0**2*(np.tan(a))**2))*(np.sqrt((1-s)**2*(np.cos(a))**2*a*fy0**2+(np.sin(a))**2*a*Cs**2)/(Cs*np.cos(a)))

    fx = fxy*fx_1
    fy = fxy*fy_1
    return [fx,fy]

s_set = np.linspace(0,1,200)
a_set = np.linspace(0,1.5,200)
Fz = 1000

fx_list = []
fy_list = []

## CHAT GPT CODE V

# Create meshgrid
S, A = np.meshgrid(s_set, a_set)
FX = np.zeros(S.shape)
FY = np.zeros(S.shape)
           
mf52 = MF52()
Cs = mf52.Fx(Fz,0.1,0)/0.1
Ca = mf52.Fy(Fz,0.1,0)/0.1

# Calculate FX and FY
for i in range(len(s_set)):
    for j in range(len(a_set)):
        fx0 = mf52.Fx(Fz,s_set[j],0)
        fy0 = mf52.Fy(Fz,a_set[i],0)
        fxy = run(s_set[i], a_set[j], fx0, fy0, Cs=Cs, Ca=Ca)
        FX[j, i] = fxy[0]
        FY[j, i] = -fxy[1]

# Plot the fx surface
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.plot_surface(S, A, FX, cmap='viridis')
ax1.set_xlim(1,0)
ax1.set_ylim(0,1.5)
ax1.set_xlabel('s')
ax1.set_ylabel('a')
ax1.set_zlabel('fx')
ax1.set_title('3D Surface of fx')

# Plot the fy surface
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
ax2.plot_surface(S, A, FY, cmap='plasma')
ax2.set_xlim(1,0)
ax2.set_ylim(0,1.5)
ax2.set_xlabel('s')
ax2.set_ylabel('a')
ax2.set_zlabel('fy')
ax2.set_title('3D Surface of fy')

# Show the plots
plt.show()

plt.show()


