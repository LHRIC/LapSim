import numpy as np
from kinematic_objects import SuspensionCorner
from kinematic_solver import kinematic_solver
from read_yaml import read_yaml

class KinematicModel:
    def __init__(self):
        pass
    def from_hardpoints(self,hardpoints_file):
        hardpoints = read_yaml(hardpoints_file)
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
    
    def _generate_model(self, corner: SuspensionCorner, steering_rack_delta, shock_travel):
        l_mid = np.linalg.norm(corner.shock_inboard.pos - corner.shock_outboard.pos)
        l_space = np.linspace(l_mid+shock_travel[0],l_mid+shock_travel[1],shock_travel[2])
        steer_space = np.linspace(steering_rack_delta[0],-steering_rack_delta[0],steering_rack_delta[1]) if steering_rack_delta is not None else np.array([0])
        shape = (len(steer_space),len(l_space))
        shape3d = (len(steer_space),len(l_space),3)

        contact_patch_positions = np.empty(shape3d)
        shock_compression = np.empty(shape)
        steer_rack_positions = np.empty(shape3d)
        wheel_poses = np.empty(shape3d)

        for i, steer in enumerate(steer_space):
            corner.inboard_tie.translate(steer,[0,1,0])
            for j, l in enumerate(l_space):
                corner.linear.length = l
                kinematic_solver(corner.dependent_objects, corner.residual_objects, corner.update_objects)
                contact_patch_positions[i,j] = corner.contact_patch.pos
                steer_rack_positions[i,j] = corner.inboard_tie.pos
                shock_compression[i,j] = corner.shock.length()
                wheel_poses[i,j] = np.rad2deg(corner.wheel_sys.delta_angle())

        delta_cp_pos = np.empty((shape[0],shape[1],3))
        delta_cp_pos_norm = np.empty(shape)
        tangent_vec = np.empty((shape[0],shape[1],3))
        delta_shock = np.empty(shape)
        motion_ratio = np.empty(shape)
        relative_shock = np.empty(shape)
        relative_steer = np.empty(shape)
        z_pos = np.empty(shape)
        surrogate_array = np.zeros((shape[0],shape[1],9))
        
        for i, steer in enumerate(steer_space):
            for j, l in enumerate(l_space):
                if j < len(l_space)-1:
                    delta_cp_pos[i,j] = contact_patch_positions[i,j] - contact_patch_positions[i,j+1]
                    delta_cp_pos_norm[i,j] = np.linalg.norm(delta_cp_pos[i,j])
                    delta_shock[i,j] = abs(shock_compression[i,j] - shock_compression[i,j+1])
                else:
                    delta_cp_pos[i,j] = delta_cp_pos[i,j-1]
                    delta_cp_pos_norm[i,j] = delta_cp_pos_norm[i,j-1]
                    delta_shock[i,j] = delta_shock[i,j-1]

                tangent_vec[i,j] = delta_cp_pos[i,j]/delta_cp_pos_norm[i,j]

                motion_ratio[i,j] = delta_cp_pos_norm[i,j]/delta_shock[i,j] if delta_shock[i,j] !=0 else 0
                relative_shock[i,j] = shock_compression[i,j] - l_mid
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
                    tangent_vec[i,j,2]                  # 8
                    ]
        return surrogate_array