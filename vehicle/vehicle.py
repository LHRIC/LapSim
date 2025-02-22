from state.state_vector import StateVector14
from state.control_vector import ControlVector
from kinematics.kinematic_model import KinematicModel
from dynamics.shock_model import ShockModel
from tires.tire_model import TireModel
from utility.read_yaml import read_yaml
from scipy.spatial.transform import Rotation as rot
import numpy as np

class Vehicle:
    def __init__(self, x0: np.ndarray, u0: np.ndarray):
        #TODO Logic to construct the models in different ways
        params = read_yaml('parameters/parameters.yaml')
        self.state_vector = StateVector14(x0)
        self.control_vector = ControlVector(u0)
        self.kinematic_model = KinematicModel()
        self.kinematic_model.from_hardpoints('parameters/hardpoints.yaml')
        self.front_shock = ShockModel(params['front_shock_stiffness'], params['front_shock_damping'])
        self.rear_shock = ShockModel(params['rear_shock_stiffness'], params['rear_shock_damping'])
        self.tire_model = TireModel()
        return
    
    def evaluate(self):
        self.state_vector.unpack()
        self.control_vector.unpack()
        vehicle_z = self.state_vector.z
        vehicle_roll = self.state_vector.roll
        vehicle_pitch = self.state_vector.pitch
        rotation_obj = rot.from_euler('xy',[vehicle_roll, vehicle_pitch])
        rotation_matrix = rotation_obj.as_matrix()
        mirror_matrix = [[1,0,0],[0,-1,0],[0,0,1]]

        for i, wheel in enumerate(['fl','fr','rl','rr']):
            z_xx = eval("self.state_vector.z_" + wheel)
            z_xx_dt = eval("self.state_vector.z_" + wheel + "_dt")
            w_xx_dt = eval("self.state_vector.w_" + wheel + "_dt")

            if wheel == 'fl' or wheel == 'fr':
                surrogate = self.kinematic_model.front
            else:
                surrogate = self.kinematic_model.rear

            kin_vec = self.kinematic_model.interpolate(z_xx, self.control_vector.steer, surrogate)
            motion_ratio = kin_vec[2]
            tangent_vec = kin_vec[6:8]
            wheel_pose = kin_vec[9:11]
            cp_pos = mirror_matrix @ kin_vec[3:6] if wheel == 'fr' or wheel == 'rr' else kin_vec[3:6]
            wheel_pose_matrix = rot.from_euler('xz', wheel_pose).as_matrix()
            
            if wheel == "fl" or "fr":
                shock = self.front_shock
            else:
                shock = self.rear_shock
            force_shock = shock.force_absolute(z_xx, z_xx_dt)
            force_uns = force_shock/motion_ratio
            cp_pos_abs = rotation_matrix @ cp_pos + [0, 0, vehicle_z]
            wheel_pose_abs = rotation_matrix @ wheel_pose_matrix
            force_tire = self.tire_model.fxyz(wheel_pose, w_xx_dt, cp_pos)
            # print(rotation_matrix)
            print(wheel, cp_pos_abs)
        return