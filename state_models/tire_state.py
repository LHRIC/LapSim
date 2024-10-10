from state_models import vehicle_state
from state_models.mf_52 import MF52 # Using Vogel_sim MF52 for now
import numpy as np
# TODO Rewrite MF52?
class TireState:
    def __init__(self,model: 'vehicle_state.VehicleState') -> None:
        self.fz = 0.0
        self.fz_elas = 0.0
        self.z = 0.0
        self.alpha = 0.0
        self.gamma = 0.0
        self.kappa = 0.0
        self.friction_scaling_x = model.params['friction_scaling_x']
        self.friction_scaling_y = model.params['friction_scaling_y']
        self.fx_max = 0.0

    def _comstock(self,model: 'vehicle_state.VehicleState'):
        
        fx_comstock = (self.fx0*self.fy0)/np.sqrt()
        fy_comstock = 1

        fx_correction = 1
        fy_correction = 1

        self.fx = 0
        self.fy = 0
        pass
    def mf52(self,model: 'vehicle_state.VehicleState'):
        #FIXME MAJOR MAJOR WIP
        mf52 = MF52()
        self.fy0 = mf52.Fy(Fz=self.fz,Alpha=self.alpha, Gamma=self.gamma)*self.friction_scaling_y
        fx_set=[]
        kappa_set = np.linspace(0,1,100)
        for kappa in np.linspace(0,1,100):
            fx_set.append(mf52.Fx(Fz=self.fz, Kappa=kappa, Gamma=self.gamma)*self.friction_scaling_x)
        if max(fx_set) <= self.fx_max: # Power limited
            self.fx0 = self.fx_max
            self.kappa = np.interp(fx_set,self.fx_max,kappa_set)
        else: # Grip limited
            idx = np.argmax(fx_set)
            self.fx0 = fx_set[idx]
            self.kappa = kappa_set[idx]

        self._comstock()
        
        self.f_vec = [self.fx,self.fy,self.fz] 