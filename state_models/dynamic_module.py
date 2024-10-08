from state_models import vehicle_state
import numpy as np
from scipy.constants import g
class DynModel:
    def __init__(self,model: 'vehicle_state.VehicleState') -> None:
        self.mass = model.params['mass']
        self.cg_bias_f = model.params['cg_bias_f']
        self.cg_height = model.params['cg_height']
        self.trackwidth_f = model.params['trackwidth_f']
        self.trackwidth_r = model.params['trackwidth_r']
        self.wheelbase = model.params['wheelbase']
        self.ride_rate_f = model.params['ride_rate_f']
        self.ride_rate_r = model.params['ride_rate_r']
        self.rollc_f = model.params['rollc_f']
        self.rollc_r = model.params['rollc_r']
        self.roll_lever_f = self.cg_height-self.rollc_f
        self.roll_lever_r = self.cg_height-self.rollc_r
        self.anti_squat = model.params['anti_squat']
        self.anti_dive = model.params['anti_dive']
        self.anti_lift = model.params['anti_lift']
        self.fl_pos = [self.cg_bias_f*self.wheelbase, self.trackwidth_f/2, 0]
        self.fr_pos = [self.cg_bias_f*self.wheelbase,-self.trackwidth_f/2, 0]
        self.rl_pos = [(self.cg_bias_f-1)*self.wheelbase, self.trackwidth_r/2, 0]
        self.rr_pos = [(self.cg_bias_f-1)*self.wheelbase, -self.trackwidth_r/2, 0]
        print(f'fl_pos {self.fl_pos}')
        print(f'fr_pos {self.fr_pos}')
        print(f'rl_pos {self.rl_pos}')
        print(f'rr_pos {self.rr_pos}')

    def static_weight(self,model: 'vehicle_state.VehicleState'):
        self.mass_f = self.mass*self.cg_bias_f
        self.mass_r = self.mass*(1-self.cg_bias_f)
        # Update Tires
        model.fl.fz += self.mass_f*g/2
        model.fr.fz += self.mass_f*g/2
        model.rl.fz += self.mass_r*g/2
        model.rr.fz += self.mass_r*g/2
        model.fl.fz_elas += self.mass_f*g/2
        model.fr.fz_elas += self.mass_f*g/2
        model.rl.fz_elas += self.mass_r*g/2
        model.rr.fz_elas += self.mass_r*g/2
        model.forces.append([0,0,-self.mass*g])

    def weight_transfer(self,model: 'vehicle_state.VehicleState'):
        k_phi_f = self.ride_rate_f*self.trackwidth_f**2*np.tan(1)/4 # F roll stiff
        k_phi_r = self.ride_rate_r*self.trackwidth_r**2*np.tan(1)/4 # R roll stiff
        k_phi_ratio = k_phi_f/(k_phi_f+k_phi_r)
        dFz_geom_roll_f = self.mass_f*self.rollc_f*model.y_ddt/self.trackwidth_f
        dFz_geom_roll_r = self.mass_r*self.rollc_r*model.y_ddt/self.trackwidth_r
        dFz_elas_roll_f = k_phi_ratio*(self.mass_f*self.roll_lever_f+self.mass_r*self.roll_lever_r)*model.y_ddt/self.trackwidth_f
        dFz_elas_roll_r = (1-k_phi_ratio)*(self.mass_f*self.roll_lever_f+self.mass_r*self.roll_lever_r)*model.y_ddt/self.trackwidth_r
        dFz_roll_f = dFz_geom_roll_f + dFz_elas_roll_f
        dFz_roll_r = dFz_geom_roll_r + dFz_elas_roll_r

        # Weight transfer from pitch
        dFz_tot_pitch = self.cg_height*self.mass_f*model.x_ddt/self.wheelbase
        if dFz_tot_pitch >= 0:
            dFz_elas_pitch_f = -dFz_tot_pitch*(1-self.anti_lift)
            dFz_elas_pitch_r = dFz_tot_pitch*(1-self.anti_squat)
        else:
            dFz_elas_pitch_f = -dFz_tot_pitch
            dFz_elas_pitch_r = dFz_tot_pitch*(1-self.anti_squat)
        dFz_geom_pitch_f = -dFz_tot_pitch + dFz_elas_pitch_f # Don't need these technically
        dFz_geom_pitch_r = dFz_tot_pitch + dFz_elas_pitch_r

        # TODO can we make this cleaner and reduce the possibility of making a sign error
        dFz_elas_fl = -dFz_elas_roll_f + dFz_elas_pitch_f/2
        dFz_elas_fr = dFz_elas_roll_f + dFz_elas_pitch_f/2
        dFz_elas_rl = -dFz_elas_roll_r + dFz_elas_pitch_r/2
        dFz_elas_rr = dFz_elas_roll_r + dFz_elas_pitch_r/2

        dFz_fl = dFz_tot_pitch - dFz_roll_f
        dFz_fr = dFz_tot_pitch + dFz_roll_f
        dFz_rl = -dFz_tot_pitch - dFz_roll_r
        dFz_rr = -dFz_tot_pitch + dFz_roll_r

        model.fl.fz += dFz_fl
        model.fr.fz += dFz_fr
        model.rl.fz += dFz_rl
        model.rr.fz += dFz_rr

        model.fl.fz_elas += dFz_elas_fl
        model.fr.fz_elas += dFz_elas_fr
        model.rl.fz_elas += dFz_elas_rl
        model.rr.fz_elas += dFz_elas_rr

        model.forces.append([model.x_ddt*self.mass,model.y_ddt*self.mass,0]) # Body forces

    def kinematic_eval(self,model: 'vehicle_state.VehicleState'):
        ride_rate_f = model.params['ride_rate_f']
        ride_rate_r = model.params['ride_rate_r']
        wheel_rate_f = model.params['wheel_rate_f']
        wheel_rate_r = model.params['wheel_rate_r']
        static_camber_f = model.params['static_camber_f']
        static_camber_r = model.params['static_camber_r']
        camber_gain_f = model.params['camber_gain_f']
        camber_gain_r = model.params['camber_gain_r']
        tire_stiff_f = wheel_rate_f*ride_rate_f/(wheel_rate_f-ride_rate_f)
        tire_stiff_r = wheel_rate_r*ride_rate_r/(wheel_rate_r-ride_rate_r)

        dz_sus_fl = model.fl.fz_elas / wheel_rate_f
        dz_sus_fr = model.fr.fz_elas / wheel_rate_f
        dz_sus_rl = model.rl.fz_elas / wheel_rate_r
        dz_sus_rr = model.rr.fz_elas / wheel_rate_r

        dz_tire_fl = model.fl.fz / tire_stiff_f
        dz_tire_fr = model.fr.fz / tire_stiff_f
        dz_tire_rl = model.rl.fz / tire_stiff_r
        dz_tire_rr = model.rr.fz / tire_stiff_r

        dz_fl = dz_sus_fl + dz_tire_fl
        dz_fr = dz_sus_fr + dz_tire_fr
        dz_rl = dz_sus_rl + dz_tire_rl
        dz_rr = dz_sus_rr + dz_tire_rr

        # Assumine front roll = rear roll & left pitch = right pitch
        model.roll = np.arcsin(((dz_fr-dz_fl)/self.trackwidth_f))
        model.pitch = np.arcsin((dz_fl-dz_rl)/self.wheelbase)

        model.fl.gamma = static_camber_f + dz_sus_fl*camber_gain_f - model.roll
        model.fr.gamma = static_camber_f + dz_sus_fr*camber_gain_f + model.roll
        model.rl.gamma = static_camber_r + dz_sus_rl*camber_gain_r - model.roll
        model.rr.gamma = static_camber_r + dz_sus_rr*camber_gain_r + model.roll
    
    def steering(self,model:'vehicle_state.VehicleState'):
        # TODO Ackermann
        static_toe_f = model.params['static_toe_f']
        static_toe_r = model.params['static_toe_r']
        psi_dt = model.y_ddt/model.v

        # Inertial frame car velocity vector
        v_vec = [model.v*np.cos(model.beta),model.v*np.sin(model.beta),0]

        # Inertial frame velocity vector of tires
        fl_vel = v_vec + np.cross(np.array([0,0,psi_dt]),np.array(self.fl_pos))
        fr_vel = v_vec + np.cross(np.array([0,0,psi_dt]),np.array(self.fr_pos))
        rl_vel = v_vec + np.cross(np.array([0,0,psi_dt]),np.array(self.rl_pos))
        rr_vel = v_vec + np.cross(np.array([0,0,psi_dt]),np.array(self.rr_pos))

        # Slip angles
        model.fl.alpha = model.delta - static_toe_f - np.arctan2(fl_vel[1],fl_vel[0])
        model.fr.alpha = model.delta + static_toe_f - np.arctan2(fr_vel[1],fr_vel[0])
        model.rl.alpha = static_toe_r - np.arctan2(rl_vel[1],rl_vel[0])
        model.rr.alpha = static_toe_r - np.arctan2(rr_vel[1],rr_vel[0])
        print(f'FL_alpha {model.fl.alpha}')
        print(f'FR_alpha {model.fr.alpha}')
        print(f'RL_alpha {model.rl.alpha}')
        print(f'RR_alpha {model.rr.alpha}')

        # Steer rotation matricies
        # TODO this will break once I add ackerman
        # TODO this ignores static toe
        x=model.delta
        self.fl_steer_mat = np.array([[np.cos(x),-np.sin(x),0],[np.sin(x),np.cos(x),0],[0,0,1]])
        self.fr_steer_mat = np.array([[np.cos(x),-np.sin(x),0],[np.sin(x),np.cos(x),0],[0,0,1]])

    def tire_forces(self,model:'vehicle_state.VehicleState'):
        
        fl_adjusted_f = np.matmul(self.fl_steer_mat,model.fl.f_vec)
        fr_adjusted_f = np.matmul(self.fr_steer_mat,model.fr.f_vec)
        
        model.forces.append(fl_adjusted_f)
        model.forces.append(fr_adjusted_f)
        model.forces.append(model.rl.f_vec)
        model.forces.append(model.rr.f_vec)

        print(f'FL_f: {fl_adjusted_f}')
        print(f'FR_f: {fr_adjusted_f}')
        print(f'RL_f: {model.rl.f_vec}')
        print(f'RR_f: {model.rr.f_vec}')
        
        fl_moment = np.cross(self.fl_pos,model.fl.f_vec)
        fr_moment = np.cross(self.fr_pos,model.fr.f_vec)
        rl_moment = np.cross(self.rl_pos,model.rl.f_vec)
        rr_moment = np.cross(self.rr_pos,model.rr.f_vec)

        print(f'FL_m: {fl_moment}')
        print(f'FR_m: {fr_moment}')
        print(f'RL_m: {rl_moment}')
        print(f'RR_m: {rr_moment}')

        model.moments.append(fl_moment)
        model.moments.append(fr_moment)
        model.moments.append(rl_moment)
        model.moments.append(rr_moment)