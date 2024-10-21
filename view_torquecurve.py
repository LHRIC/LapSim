from matplotlib import pyplot as plt
import numpy as np
file_path = 'vehicles/23_torque_curve.csv'
torque_curve = np.loadtxt(file_path, delimiter=',', unpack=True)
interp_array = np.linspace(0,15000,20)
torque_curve2 = np.interp(interp_array,torque_curve[0],torque_curve[1])
# plt.plot(torque_curve[0],torque_curve[1])
# plt.plot(interp_array,torque_curve2)
# plt.show()

v_set = np.linspace(2.5,20,1000)
tire_radius = 0.2032
primary_reduction = 76/36
final_drive = 37/11
shiftpoint = 12500
gear_ratios = {1: 33/12, 2: 32/16, 3: 30/18, 4: 26/18, 5: 30/23, 6: 29/24}
torque_list = []
rpm_list = []
gear_list=[]
for v in v_set:
    wheel_rpm = 60*v / (2*np.pi*tire_radius)
    raw_shaft_rpm = wheel_rpm*primary_reduction*final_drive
    gear = 1
    shaft_rpm = raw_shaft_rpm*gear_ratios[1]
    while shaft_rpm > shiftpoint and gear <= max(gear_ratios.keys()):
        # print ('shifting')
        gear += 1
        shaft_rpm = raw_shaft_rpm*gear_ratios[gear]
    # print(f'gear: {gear} raw rpm {raw_shaft_rpm} adj_rpm {shaft_rpm}')
    max_torque = np.interp(shaft_rpm,torque_curve[0],torque_curve[1])
    torque_list.append(max_torque)
    rpm_list.append(shaft_rpm)
    gear_list.append(gear)
plt.plot(v_set,torque_list)
plt.show()
        