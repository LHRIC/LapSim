from kinematics.kinematic_model import KinematicModel
# from kinematics.multibody_kinematics import integrate_roll_axis
import kinematics.multibody_kinematics as mkb
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.ticker as FuncFormatter
import numpy as np

kin_model = KinematicModel()
# kin_model.from_hardpoints('parameters/hardpoints.yaml')
steering_delta = [20, 51]
front_left_delta = [-12, 12, 51]
rear_left_delta = [-12, 12, 51]
kin_model.from_xlsx('parameters/HDPT_Export.xlsx', steering_delta, front_left_delta, rear_left_delta)
f = kin_model.front
r = kin_model.rear

rc_z = mkb.integrate_roll_axis(kin_model, 0.0, 1.0, 100, [0,0], 1)
print(rc_z)