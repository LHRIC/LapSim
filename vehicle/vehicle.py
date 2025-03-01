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
        vehicle_mass = params['vehicle_mass']
        self.radius = 0.406 # Pull from params
        return
    
    def _slip_angle(self, wheel_pose_abs, local_wheel_vel):
        wheel_heading_xy = wheel_pose_abs[:2,0]
        wheel_vel_xy = local_wheel_vel[:2]
        heading_dot = np.dot(wheel_heading_xy, wheel_vel_xy)
        norm_product = np.linalg.norm(wheel_heading_xy)*np.linalg.norm(wheel_vel_xy)
        slip_angle = np.arccos(heading_dot/norm_product)
        return slip_angle
    
    def _slip_ratio(self, wheel_pose_abs, local_wheel_vel, w_xx_dt):
        wheel_heading_xy_u = wheel_pose_abs[:2,0]/np.linalg.norm(wheel_pose_abs[:2,0])
        wheel_vel_xy = local_wheel_vel[:2]
        wheel_vel_relx = np.dot(wheel_vel_xy, wheel_heading_xy_u)
        tangential_vel = w_xx_dt*self.radius
        slip_ratio = tangential_vel/wheel_vel_relx
        return slip_ratio

    def evaluate(self):
        self.state_vector.unpack()
        self.control_vector.unpack()
        vehicle_xy_vel = [self.state_vector.x_dt, self.state_vector.y_dt]
        vehicle_z = self.state_vector.z
        vehicle_roll = self.state_vector.roll
        vehicle_pitch = self.state_vector.pitch
        vehicle_yaw_dt = self.state_vector.yaw_dt
        rotation_obj = rot.from_euler('xy',[vehicle_roll, vehicle_pitch])
        rotation_matrix = rotation_obj.as_matrix()
        mirror_matrix = np.array([[1,0,0],[0,-1,0],[0,0,1]])
        corner_forces = np.zeros((4,3))
        z_ddt_vec = np.zeros(4)
        corner_torques = np.zeros((4,3))

        for i, wheel in enumerate(['fl','fr','rl','rr']):
            z_xx = eval("self.state_vector.z_" + wheel)
            z_xx_dt = eval("self.state_vector.z_" + wheel + "_dt")
            w_xx_dt = eval("self.state_vector.w_" + wheel + "_dt")
            xx_mass = 10 ####### TODO

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
    
            force_uns_shock = force_shock/motion_ratio
            cp_pos_abs = rotation_matrix @ cp_pos + [0, 0, vehicle_z]
            wheel_pose_abs = rotation_matrix @ wheel_pose_matrix
            wheel_pose_abs_euler = rot.from_matrix(wheel_pose_matrix).as_euler('xyz')
            force_shock = shock.force_absolute(z_xx, z_xx_dt)
            tire_displacement = -np.clip(cp_pos_abs[2], None, 0)
            yaw_rate_matrix = rot.from_euler('z', vehicle_yaw_dt).as_matrix() #### #TODO DOES THIS MAKE SENSE??
            local_wheel_vel = yaw_rate_matrix @ cp_pos_abs + [self.state_vector.x_dt, self.state_vector.y_dt, 0]
            slip_angle = self._slip_angle(wheel_pose_abs, local_wheel_vel)
            slip_ratio = self._slip_ratio(wheel_pose_abs, local_wheel_vel, w_xx_dt)
            inclination_angle = wheel_pose_abs_euler[1]
            force_tire = self.tire_model.fxyz(tire_displacement, slip_angle, slip_ratio, inclination_angle)
            force_tire_tangent = np.dot(force_tire, tangent_vec)
            force_tire_compliment = force_tire = force_tire_tangent
            force_uns = force_tire_tangent - force_uns_shock
            z_ddt_vec[i] = force_uns/xx_mass
            corner_forces[i] = force_tire_compliment + force_uns_shock*tangent_vec
            corner_torques[i] = np.cross(cp_pos_abs, corner_forces[i])
                    
            
        # End of outboard component analysis
        

        return