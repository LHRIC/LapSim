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
        self.front_shock = ShockModel(params['front_shock_stiffness'], params['front_shock_damping'], params['front_shock_ride_compression'])
        self.rear_shock = ShockModel(params['rear_shock_stiffness'], params['rear_shock_damping'], params['rear_shock_ride_compression'])
        self.tire_model = TireModel()
        self.vehicle_mass = params['vehicle_mass']
        self.vehicle_inertia = params['vehicle_inertia']
        self.radius = 0.406 # Pull from params
        self.x_dot = np.zeros(len(x0))
        self.center_of_mass = params['center_of_mass']
        return

    def _slip_angle(self, wheel_pose_abs, local_wheel_vel):
        wheel_heading_xy = wheel_pose_abs[:2,0]
        wheel_vel_xy = local_wheel_vel[:2]
        heading_dot = np.dot(wheel_heading_xy, wheel_vel_xy)
        norm_product = np.linalg.norm(wheel_heading_xy)*np.linalg.norm(wheel_vel_xy)
        slip_angle = np.arccos(heading_dot/norm_product) if norm_product != 0 else 0
        return slip_angle

    def _slip_ratio(self, wheel_pose_abs, local_wheel_vel, w_xx_dt):
        wheel_heading_xy_u = wheel_pose_abs[:2,0]/np.linalg.norm(wheel_pose_abs[:2,0])
        wheel_vel_xy = local_wheel_vel[:2]
        wheel_vel_relx = np.dot(wheel_vel_xy, wheel_heading_xy_u)
        tangential_vel = w_xx_dt*self.radius
        slip_ratio = tangential_vel/wheel_vel_relx if wheel_vel_relx !=0 else 0
        return slip_ratio

    # @profile
    def evaluate(self):
        self.state_vector.unpack()
        self.control_vector.unpack()
        vehicle_z = self.state_vector.z
        vehicle_roll = self.state_vector.roll
        vehicle_pitch = self.state_vector.pitch
        vehicle_yaw_dt = self.state_vector.yaw_dt
        angular_velocity_vec = np.array([self.state_vector.roll_dt, self.state_vector.pitch_dt, self.state_vector.yaw_dt])
        # rotation_obj = rot.from_euler('xy',[vehicle_roll, vehicle_pitch])
        # rotation_obj = rot.from_euler('xy',[0,0])
        rotation_obj = rot.from_rotvec([vehicle_roll, vehicle_pitch, 0])
        rotation_matrix = rotation_obj.as_matrix()
        mirror_matrix = np.array([[1,0,0],[0,-1,0],[0,0,1]])
        corner_forces = np.zeros((4,3))
        z_ddt_vec = np.zeros(4)
        corner_torques = np.zeros((4,3))
        center_of_mass = rotation_matrix @ self.center_of_mass
        z_xx_list = [self.state_vector.z_fl, self.state_vector.z_fr, self.state_vector.z_rl, self.state_vector.z_rr]
        z_xx_dt_list = [self.state_vector.z_fl_dt, self.state_vector.z_fr_dt, self.state_vector.z_rl_dt, self.state_vector.z_rr_dt]
        w_xx_dt_list = [self.state_vector.w_fl_dt, self.state_vector.w_fr_dt, self.state_vector.w_rl_dt, self.state_vector.w_rr_dt]

        for i, wheel in enumerate(['fl','fr','rl','rr']):
            z_xx = z_xx_list[i]
            z_xx_dt =z_xx_dt_list[i]
            w_xx_dt = w_xx_dt_list[i]
            xx_mass = 7 ####### TODO

            if wheel == 'fl' or wheel == 'fr':
                interpolator = self.kinematic_model.front_interpolator
            else:
                interpolator = self.kinematic_model.rear_interpolator

            kin_vec = self.kinematic_model.interpolate(z_xx, self.control_vector.steer, interpolator)
            motion_ratio = kin_vec[2]
            motion_ratio = 1 ##### REMOVE
            tangent_vec_local = mirror_matrix @ kin_vec[6:9] if wheel == 'fr' or wheel == 'rr' else kin_vec[6:9]
            tangent_vec = rotation_matrix @ tangent_vec_local
            tangent_vec = np.array([0,0,1]) #### REMOVE
            wheel_pose = kin_vec[9:11]
            cp_pos = mirror_matrix @ kin_vec[3:6] if wheel == 'fr' or wheel == 'rr' else kin_vec[3:6]
            wheel_pose_matrix = rot.from_euler('xz', wheel_pose).as_matrix()
            wheel_pose_matrix = mirror_matrix @ wheel_pose_matrix if wheel == 'fr' or wheel == 'rr' else wheel_pose_matrix
            
            if wheel == 'fl' or wheel == 'fr':
                shock = self.front_shock
            else:
                shock = self.rear_shock
    
            cp_pos_abs = rotation_matrix @ cp_pos + [0, 0, vehicle_z]
            wheel_pose_abs = rotation_matrix @ wheel_pose_matrix
            wheel_pose_abs_euler = rot.from_matrix(wheel_pose_matrix).as_euler('xyz')
            tire_displacement = -np.clip(cp_pos_abs[2], a_min=None, a_max=0)
            # tire_displacement = -cp_pos_abs[2]
            yaw_rate_vec = np.array([0, 0, vehicle_yaw_dt])
            local_wheel_vel = np.cross(cp_pos_abs, yaw_rate_vec) + [self.state_vector.x_dt, self.state_vector.y_dt, 0]
            slip_angle = self._slip_angle(wheel_pose_abs, local_wheel_vel)
            slip_ratio = self._slip_ratio(wheel_pose_abs, local_wheel_vel, w_xx_dt)
            inclination_angle = wheel_pose_abs_euler[1]

            # Forces acting on unsprung
            force_damp, force_spring = shock.force_absolute(-z_xx, z_xx_dt) # Positive force = negative z_xx (compression)
            force_shock = -force_damp + force_spring
            force_uns_shock = force_shock/motion_ratio
            # force_tire = self.tire_model.fxyz(tire_displacement, slip_angle, slip_ratio, inclination_angle)
            force_tire = self.tire_model.fz(tire_displacement, slip_angle, slip_ratio, inclination_angle)
            force_uns_gravity = np.array([0,0,-9.81])*xx_mass
            sum_force_uns = force_tire + force_uns_gravity
            sum_force_uns_proj = np.dot(sum_force_uns, tangent_vec) # Projection of sum force on velocity vector: scalar
            sum_force_uns_orth = sum_force_uns - sum_force_uns_proj*tangent_vec # Orthagonal component of sum force wrt. velocity vector: vector r^3

            # print(force_shock, force_tire)

            z_ddt_vec[i] = (force_uns_shock - sum_force_uns_proj)*motion_ratio/xx_mass # ACCOUNT FOR MOTION RATIO PLZ
            corner_forces[i] = sum_force_uns_orth + force_uns_shock*tangent_vec
            corner_torques[i] = np.cross(cp_pos_abs, corner_forces[i])
        
        # End of outboard component analysis

        gravity_force = self.vehicle_mass * np.array([0,0,-9.81])
        gravity_torque = np.cross(center_of_mass, gravity_force)
        sum_forces = corner_forces.sum(axis=0) + gravity_force
        sum_torques = corner_torques.sum(axis=0) + gravity_torque
        accel_vec =  sum_forces/self.vehicle_mass
        rot_accel_vec = np.linalg.inv(self.vehicle_inertia) @ (sum_torques - np.cross(angular_velocity_vec, self.vehicle_inertia @ angular_velocity_vec))
        w_ddt_vec = np.zeros(4) #TODO

        self.x_dot[0:14] = self.state_vector.state[14:28]
        self.x_dot[14:17] = accel_vec*1000
        self.x_dot[17:20] = rot_accel_vec
        self.x_dot[20:24] = z_ddt_vec*1000
        self.x_dot[24:28] = w_ddt_vec

        return self.x_dot