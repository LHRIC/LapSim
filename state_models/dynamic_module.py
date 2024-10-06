from state_models import vehicle_state
import numpy as np
from scipy.constants import g
class Dynamic_module:
    def __init__(self,model: 'vehicle_state.Vehicle_state') -> None:
        self.mass = model.params['mass']
        self.cg_bias_f = model.params['cg_bias_f']
        self.cg_height = model.params['cg_height']
        self.trackwidth_f = model.params['trackwidth_f']
        self.trackwidth_r = model.params['trackwidth_r']
        self.wheelbase = model.params['wheelbase']
        self.ride_rate_f = model.params['ride_rate_f']
        self.ride_rate_r = model.params['ride_rate_r']
        self.rollc_f = model.params['rollc_f']
        self.rollc_r = model.params['rollc_f']
        self.roll_lever_f = self.cg_height-self.rollc_f
        self.roll_lever_r = self.cg_height-self.rollc_r
        self.anti_squat = model.params['anti_squat']
        self.anti_dive = model.params['anti_dive']
        self.anti_lift = model.params['anti_lift']

    def static_weight(self,model: 'vehicle_state.Vehicle_state'):
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
    
    def weight_transfer(self,model: 'vehicle_state.Vehicle_state'):
        k_phi_f = self.ride_rate_f*self.trackwidth_f**2*np.tan(1)/4 # F roll stiff
        k_phi_r = self.ride_rate_r*self.trackwidth_r**2*np.tan(1)/4 # R roll stiff
        k_phi_ratio = k_phi_f/k_phi_r
        print(f'k phi ratio {k_phi_ratio}')
        print(f'mass front/rear {self.mass_f} {self.mass_r}')
        print(f'rollc front/rear {self.rollc_f} {self.rollc_r}')
        dFz_geom_roll_f = self.mass_f*g*self.rollc_f*model.y_ddt/self.trackwidth_f
        dFz_geom_roll_r = self.mass_r*g*self.rollc_r*model.y_ddt/self.trackwidth_r
        dFz_elas_roll_f = k_phi_ratio*(self.mass_f*g*self.roll_lever_f+self.mass_r*g*self.roll_lever_r)*model.y_ddt/self.trackwidth_f
        dFz_elas_roll_r = (1-k_phi_ratio)*(self.mass_f*g*self.roll_lever_f+self.mass_r*g*self.roll_lever_r)*model.y_ddt/self.trackwidth_r
        dFz_roll_f = dFz_geom_roll_f + dFz_elas_roll_f
        dFz_roll_r = dFz_geom_roll_r + dFz_elas_roll_r
        dFz_roll_f2 = model.y_ddt*self.mass_f*g*self.cg_height/self.trackwidth_f
        print(f'lat weight transfer f_elas, r_elas, f_geom, r_geom {dFz_elas_roll_f} {dFz_elas_roll_r} {dFz_geom_roll_f} {dFz_geom_roll_r}')
        print(f'lat weight transfer alt {dFz_roll_f2}')
        print(f'lat weight trasnfer {dFz_roll_f} {dFz_roll_r}')
        # Weight transfer from pitch
        dFz_tot_pitch = self.cg_height*self.mass_f*g*model.x_ddt/self.wheelbase
        print(f'long weight transfer: {dFz_tot_pitch}')
        if dFz_tot_pitch >= 0:
            dFz_elas_pitch_f = -dFz_tot_pitch*(1-self.anti_lift)
            dFz_elas_pitch_r = dFz_tot_pitch*(1-self.anti_squat)
        else:
            dFz_elas_pitch_f = -dFz_tot_pitch
            dFz_elas_pitch_r = dFz_tot_pitch*(1-self.anti_squat)
        dFz_geom_pitch_f = -dFz_tot_pitch + dFz_elas_pitch_f
        dFz_geom_pitch_r = dFz_tot_pitch + dFz_elas_pitch_r

        # TODO can we make this cleaner
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

    def kinematic_eval(self,model: 'vehicle_state.Vehicle_state'):
        pass
    
    
    