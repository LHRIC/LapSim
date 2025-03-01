import numpy as np
from utility.read_yaml import read_yaml
from scipy.interpolate import RegularGridInterpolator

class KinematicModel:
    def __init__(self):
        pass
    def from_parameters(self, param_file):
        params = read_yaml(param_file)
        self.steering_rack_delta = params["steering_rack_delta"]
        self.front_shock_travel = params["front_shock_travel"] 
        self.rear_shock_travel = params["rear_shock_travel"] 
        self.trackwidth_front = params["trackwidth_front"] 
        self.trackwidth_rear = params["trackwidth_rear"] 
        self.motion_ratio_front = params["motion_ratio_front"] 
        self.motion_ratio_rear = params["motion_ratio_rear"] 
        self.nonlin_motional_ratio_front = params["nonlin_motional_ratio_front"]
        self.nonlin_motional_ratio_rear = params["nonlin_motional_ratio_rear"] 
        self.front_camber = params["front_camber_gain"] 
        self.rear_camber = params["rear_camber_gain"] 
        self.bump_steer_front = params["bump_steer_front"]
        self.bump_steer_rear = params["bump_steer_rear"] 
        self.wheelbase = params["wheelbase"]
        self.steer_rate = params["steer_rate"]
        self.ackermann = params["ackermann"]
        self.front = self._generate_model(self.front_shock_travel, self.steering_rack_delta, 
                                          self.front_camber, self.bump_steer_front, 
                                          self.trackwidth_front, self.motion_ratio_front,
                                          self.nonlin_motional_ratio_front)
        self.rear = self._generate_model(self.rear_shock_travel, self.steering_rack_delta, 
                                         self.rear_camber, self.bump_steer_rear,
                                         self.trackwidth_rear, self.motion_ratio_rear, 
                                         self.nonlin_motional_ratio_rear)

    def contact_patch(self, track_width, wheelbase, shock, motion_ratio):
        x = track_width / 2
        y = wheelbase / 2
        z = shock / motion_ratio

        return np.array([x, y, z])

    
    def _generate_model(self, shock, steer, camber_gain, 
                        bump_steer, trackwidth, motion_ratio_num, nonlin_motion_ratio):
        shock_space = np.linspace(shock[0], shock[1],shock[2])
        steer_space = np.linspace(-steer[0],steer[0],steer[1]) if steer is not None else np.array([0])
        shape = (len(shock_space),len(steer_space))
        shape3d = (len(shock_space),len(steer_space),3)

        relative_shock = np.empty(shape)
        relative_steer = np.empty(shape)
        motion_ratio = np.empty(shape)
        contact_patch_positions = np.empty(shape3d)
        delta_cp_pos = np.empty((shape[0],shape[1],3))
        delta_cp_pos_norm = np.empty(shape)
        delta_shock = np.empty(shape)
        tangent_vec = np.empty((shape[0],shape[1],3))
        camber = np.empty(shape)
        toe = np.empty(shape)

        surrogate_array = np.zeros((len(shock_space), len(steer_space), 11)) 

        shock_length = abs(shock[0] - shock[1])
        num_steps = shock[2]

        step_size = shock_length / num_steps
        # print("step size", step_size)

        for i, shock in enumerate(shock_space):
            for j, steer in enumerate(steer_space):
                contact_patch_positions[i, j] = np.array([
                        trackwidth / 2, 
                        self.wheelbase / 2, 
                        i / motion_ratio_num ])
                        # if motion_ratio_num != 0 else 0])
                # self.contact_patch(self.trackwidth, self.wheelbase, self.front_shock_travel[i, j])

                if i < len(shock_space)-1:
                    delta_cp_pos[i,j] = contact_patch_positions[i,j] - contact_patch_positions[i+1,j]
                    delta_cp_pos_norm[i,j] = np.linalg.norm(delta_cp_pos[i,j])
                    # step size 
                    delta_shock[i,j] = abs(shock_space[i] - shock_space[i+1])
                else:
                    delta_cp_pos[i,j] = delta_cp_pos[i-1,j]
                    delta_cp_pos_norm[i,j] = delta_cp_pos_norm[i-1,j]
                    delta_shock[i,j] = delta_shock[i-1,j]

                relative_shock[i, j] = step_size
                if j < len(steer_space)-1:
                    relative_steer[i, j] = abs(steer_space[j] - steer_space[j+1])
                else:
                    print("else")
                    relative_steer[i, j] = abs(steer_space[j] - steer_space[j-1])
                    
                motion_ratio[i, j] = motion_ratio_num + (nonlin_motion_ratio * shock_space[i])
                tangent_vec[i,j] = delta_cp_pos[i,j]/delta_cp_pos_norm[i,j]
                camber[i] = camber_gain * relative_shock[i]
                toe[j] = bump_steer * relative_shock[i] + steer_space[j] * self.steer_rate

                surrogate_array[i,j] = [                # Index:
                    relative_shock[i,j],                # 0
                    relative_steer[i,j],                # 1
                    motion_ratio[i,j],                  # 2
                    contact_patch_positions[i,j,0],     # 3
                    contact_patch_positions[i,j,1],     # 4
                    contact_patch_positions[i,j,2],     # 5
                    tangent_vec[i,j,0],                 # 6 0
                    tangent_vec[i,j,1],                 # 7 0
                    tangent_vec[i,j,2],                 # 8 1
                    camber[i,0],                 # 9 
                    toe[j,2]                  # 10
                    ]
                
        print(surrogate_array)
        return surrogate_array
    
    def interpolate(self, relative_shock, relative_steer, surrogate_array):
        shock_space = surrogate_array[:,0,0]
        steer_space = (surrogate_array[0,:,1]).T
        interp = RegularGridInterpolator((shock_space,steer_space),surrogate_array,fill_value=np.nan)
        return interp(np.array([relative_shock,relative_steer]))