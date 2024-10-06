from state_models import vehicle_state
class Aero_module:
    def __init__(self,model: 'vehicle_state.Vehicle_state') -> None:
        self.cla=model.params['cla']
        self.cda=model.params['cda']
        self.cop=model.params['cop']
        self.rho = 1.293 # density of air in kg/m^3
    def downforce(self,model: 'vehicle_state.Vehicle_state'):
        dFz_aero_f = self.rho/2*self.cop*self.cla*model.v**2
        dFz_aero_r = self.rho/2*(1-self.cop)*self.cla*model.v**2
        
        model.fl.fz += dFz_aero_f
        model.fr.fz += dFz_aero_f
        model.rl.fz += dFz_aero_r
        model.rr.fz += dFz_aero_r

        model.fl.fz_elas += dFz_aero_f
        model.fr.fz_elas += dFz_aero_f
        model.rl.fz_elas += dFz_aero_r
        model.rr.fz_elas += dFz_aero_r

    def drag(self,model: 'vehicle_state.Vehicle_state'):
        pass