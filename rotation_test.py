import numpy as np
from scipy.spatial.transform import Rotation as rot


vehicle_roll = np.deg2rad(45)
vehicle_pitch = np.deg2rad(15)
rotation_obj = rot.from_euler('xy',[vehicle_roll, vehicle_pitch])
rotation_matrix = rotation_obj.as_matrix()
print(rotation_matrix)