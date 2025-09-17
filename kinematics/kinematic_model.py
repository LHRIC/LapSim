import numpy as np
from kinematics.kinematic_objects import SuspensionCorner
from kinematics.kinematic_solver import kinematic_solver
from utility.read_yaml import read_yaml
from utility.read_xlsx import read_xlsx
from scipy.interpolate import RegularGridInterpolator
from kinematics.roll_centers import cp_roll_projection

class KinematicModel:
    def __init__(self):
        pass
    def from_hardpoints(self, hardpoints_file):
        hardpoints = read_yaml(hardpoints_file)
        origin = ((np.array(hardpoints['front_left']['FLWC']) + np.array(hardpoints['rear_left']['RLWC']))/2)[0]
        for key, value in hardpoints['front_left'].items():
            if value != []:
                hardpoints['front_left'][key][0] = value[0] - origin
        for key, value in hardpoints['rear_left'].items():
            if value != []:
                hardpoints['rear_left'][key][0] = value[0] - origin

        self.steering_rack_delta = hardpoints['steering_rack_delta']
        self.front_shock_travel = hardpoints['front_shock_travel']
        self.rear_shock_travel = hardpoints['rear_shock_travel']
        self.front_arb_exists = hardpoints['front_arb_exists']
        self.rear_arb_exists = hardpoints['rear_arb_exists']
        def _snip(corner_dict):
            hardpoints_snipped = {}
            for key, value in corner_dict.items():
                new_key = key[2:]
                hardpoints_snipped[new_key] = value
            return hardpoints_snipped
        self.fl_hardpoints = _snip(hardpoints['front_left'])
        self.rl_hardpoints = _snip(hardpoints['rear_left'])
        self.front_left = SuspensionCorner(self.fl_hardpoints, self.front_arb_exists)
        self.rear_left = SuspensionCorner(self.rl_hardpoints, self.rear_arb_exists)
        self.front = self._generate_model(self.front_left, self.steering_rack_delta, self.front_shock_travel)
        self.rear = self._generate_model(self.rear_left, None, self.rear_shock_travel)
        self.front_interpolator = self._init_interp(self.front)
        self.rear_interpolator = self._init_interp(self.rear)
        np.save('front_kin_surrogate', self.front)
        np.save('rear_kin_surrogate', self.rear)

    def from_xlsx(self, hardpoints_file, steering_delta, front_shock_delta, rear_shock_delta):
        hardpoints = read_xlsx(hardpoints_file)
        hardpoints['steering_rack_delta'] = steering_delta
        hardpoints['front_shock_travel'] = front_shock_delta
        hardpoints['rear_shock_travel'] = rear_shock_delta
        hardpoints['front_arb_exists'] = False
        hardpoints['rear_arb_exists'] = False

        origin = ((np.array(hardpoints['front_left']['FLWC']) + np.array(hardpoints['rear_left']['RLWC']))/2)[0]
        for key, value in hardpoints['front_left'].items():
            if value != []:
                hardpoints['front_left'][key][0] = value[0] - origin
        for key, value in hardpoints['rear_left'].items():
            if value != []:
                hardpoints['rear_left'][key][0] = value[0] - origin

        self.steering_rack_delta = hardpoints['steering_rack_delta']
        self.front_shock_travel = hardpoints['front_shock_travel']
        self.rear_shock_travel = hardpoints['rear_shock_travel']
        self.front_arb_exists = hardpoints['front_arb_exists']
        self.rear_arb_exists = hardpoints['rear_arb_exists']
        def _snip(corner_dict):
            hardpoints_snipped = {}
            for key, value in corner_dict.items():
                new_key = key[2:]
                hardpoints_snipped[new_key] = value
            return hardpoints_snipped
        self.fl_hardpoints = _snip(hardpoints['front_left'])
        self.rl_hardpoints = _snip(hardpoints['rear_left'])
        self.front_left = SuspensionCorner(self.fl_hardpoints, self.front_arb_exists)
        self.rear_left = SuspensionCorner(self.rl_hardpoints, self.rear_arb_exists)
        self.front = self._generate_model(self.front_left, self.steering_rack_delta, self.front_shock_travel)
        self.rear = self._generate_model(self.rear_left, None, self.rear_shock_travel)
        self.front_interpolator = self._init_interp(self.front)
        self.rear_interpolator = self._init_interp(self.rear)
        np.save('front_kin_surrogate', self.front)
        np.save('rear_kin_surrogate', self.rear)


    
    def _generate_model(self, corner: SuspensionCorner, steering_rack_delta, shock_travel):
        shock_mid = np.linalg.norm(corner.shock_inboard.pos - corner.shock_outboard.pos)
        shock_space = np.linspace(shock_mid+shock_travel[0],shock_mid+shock_travel[1],shock_travel[2])
        steer_space = np.linspace(-steering_rack_delta[0],steering_rack_delta[0],steering_rack_delta[1]) if steering_rack_delta is not None else np.array([0])
        shape = (len(shock_space),len(steer_space))
        shape3d = (len(shock_space),len(steer_space),3)

        contact_patch_positions = np.empty(shape3d)
        shock_compression = np.empty(shape)
        steer_rack_positions = np.empty(shape3d)
        wheel_poses = np.empty(shape3d)
        instant_center = np.empty(shape3d)

        for i, shock in enumerate(shock_space):
            corner.linear.length = shock
            for j, steer in enumerate(steer_space):
                corner.inboard_tie.translate(steer,[0,1,0])
                kinematic_solver(corner.dependent_objects, corner.residual_objects, corner.update_objects)
                contact_patch_positions[i,j] = corner.contact_patch.pos
                steer_rack_positions[i,j] = corner.inboard_tie.pos
                shock_compression[i,j] = corner.shock.length()
                wheel_poses[i,j] = corner.wheel_sys.delta_angle()
                instant_center[i,j] = cp_roll_projection(corner.lower_outboard.pos, 
                                                         corner.lower_inboard_fore.pos, 
                                                         corner.lower_inboard_aft.pos,
                                                         corner.upper_outboard.pos,
                                                         corner.upper_inboard_fore.pos,
                                                         corner.upper_inboard_aft.pos,
                                                         corner.contact_patch.pos)
        delta_cp_pos = np.empty((shape[0],shape[1],3))
        delta_cp_pos_norm = np.empty(shape)
        tangent_vec = np.empty((shape[0],shape[1],3))
        delta_shock = np.empty(shape)
        motion_ratio = np.empty(shape)
        relative_shock = np.empty(shape)
        relative_steer = np.empty(shape)

        ### UPDATE SHAPE IF ADDING VARIABLES ###
        surrogate_array = np.zeros((shape[0],shape[1],14)) 
        ### -------------------------------- ###
        
        for i, shock in enumerate(shock_space):
            for j, steer in enumerate(steer_space):
                if i < len(shock_space)-1:
                    delta_cp_pos[i,j] = contact_patch_positions[i,j] - contact_patch_positions[i+1,j]
                    delta_cp_pos_norm[i,j] = np.linalg.norm(delta_cp_pos[i,j])
                    delta_shock[i,j] = abs(shock_compression[i,j] - shock_compression[i+1,j])
                else:
                    delta_cp_pos[i,j] = delta_cp_pos[i-1,j]
                    delta_cp_pos_norm[i,j] = delta_cp_pos_norm[i-1,j]
                    delta_shock[i,j] = delta_shock[i-1,j]

                tangent_vec[i,j] = delta_cp_pos[i,j]/delta_cp_pos_norm[i,j]

                motion_ratio[i,j] = delta_shock[i,j]/delta_cp_pos_norm[i,j] if delta_cp_pos_norm[i,j] !=0 else 0
                relative_shock[i,j] = shock_compression[i,j] - shock_mid
                relative_steer[i,j] = steer_rack_positions[i,j,1] - corner.inboard_tie.initial_pos[1]
                
                surrogate_array[i,j] = [                # Index:
                    relative_shock[i,j],                # 0
                    relative_steer[i,j],                # 1
                    motion_ratio[i,j],                  # 2
                    contact_patch_positions[i,j,0],     # 3
                    contact_patch_positions[i,j,1],     # 4
                    contact_patch_positions[i,j,2],     # 5
                    tangent_vec[i,j,0],                 # 6
                    tangent_vec[i,j,1],                 # 7
                    tangent_vec[i,j,2],                 # 8
                    wheel_poses[i,j,0],                 # 9
                    wheel_poses[i,j,2],                 # 10
                    instant_center[i,j,0],              # 11
                    instant_center[i,j,1],              # 12
                    instant_center[i,j,2]               # 13
                    ]
        return surrogate_array
    
    def _init_interp(self, surrogate_array):
        self.shock_space = surrogate_array[:,0,0]
        self.steer_space = (surrogate_array[0,:,1]).T
        interp = RegularGridInterpolator((self.shock_space, self.steer_space), surrogate_array, bounds_error=True, fill_value=None, method="linear")
        return interp
    
    def _init_z_interp(self, surrogate_array):
        middle_steer_idx = len((surrogate_array[0,:,1]).T)//2
        z_axis_input = surrogate_array[:,middle_steer_idx,5]
        interp = RegularGridInterpolator((z_axis_input), np.squeeze(surrogate_array[:,middle_steer_idx,:]))
        return interp
    
    def interpolate(self, relative_shock, relative_steer, interp):
        clip_shock = np.clip(relative_shock, self.shock_space[0], self.shock_space[-1])
        clip_steer = np.clip(relative_steer, self.steer_space[0], self.steer_space[-1])
        result = interp(np.array([clip_shock, clip_steer]))[0]
        # interp returns an array as it takes in an array of values to interpolate at so we take... 
        # ...the first element b/c we only interpolate at one point
        return result
