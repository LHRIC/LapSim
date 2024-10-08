from state_models import vehicle_state
import numpy as np
class AeroModel:
    def __init__(self,model: 'vehicle_state.VehicleState') -> None:
        self.cla = model.params['cla']
        self.cda = model.params['cda']
        self.cop = model.params['cop']
        self.wheelbase = model.params['wheelbase']
        self.cg_bias_f = model.params['cg_bias_f']
        self.rho = 1.293 # density of air in kg/m^3
        
    def downforce(self,model: 'vehicle_state.VehicleState'):
        fz_aero_f = self.rho/2*self.cop*self.cla*model.v**2
        fz_aero_r = self.rho/2*(1-self.cop)*self.cla*model.v**2
        
        model.fl.fz += fz_aero_f/2
        model.fr.fz += fz_aero_f/2
        model.rl.fz += fz_aero_r/2
        model.rr.fz += fz_aero_r/2

        model.fl.fz_elas += fz_aero_f/2
        model.fr.fz_elas += fz_aero_f/2
        model.rl.fz_elas += fz_aero_r/2
        model.rr.fz_elas += fz_aero_r/2

        downforce_vec = np.array([0,0,-fz_aero_f-fz_aero_r])
        downforce_pos = np.array([self.wheelbase*(self.cop-self.cg_bias_f),0,0])
        downforce_moment = np.linalg.cross(downforce_pos,downforce_vec)
        model.forces.append(downforce_vec) 
        model.moments.append(downforce_moment) 

    def drag(self,model: 'vehicle_state.VehicleState'):
        pass