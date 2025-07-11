import numpy as np
from scipy.spatial.transform import Rotation as rot


vehicle_roll = np.deg2rad(45)
vehicle_pitch = np.deg2rad(45)
rotation_obj = rot.from_euler('xy',[vehicle_roll, vehicle_pitch])
rotation_obj2 = rot.from_euler('x', vehicle_roll)
rotation_obj3 = rot.from_euler('y', vehicle_pitch)
rotation_matrix = rotation_obj.as_matrix()
rotation_matrix2 = rotation_obj2.as_matrix()
rotation_matrix3 = rotation_obj3.as_matrix()
print(rotation_matrix)
print(rotation_matrix2)
print(rotation_matrix3)
print(rotation_matrix3 @ rotation_matrix2)

