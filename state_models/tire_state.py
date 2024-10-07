from state_models import vehicle_state
from state_models.MF52 import MF52 # Using Vogel_sim MF52 for now
import numpy as np
# TODO Rewrite MF52?
class Tire_state:
    def __init__(self,model: 'vehicle_state.Vehicle_state') -> None:
        self.fz = 0.0
        self.fz_elas = 0.0
        self.z = 0.0
        self.alpha = 0.0
        self.gamma = 0.0
    def mf52(self,model: 'vehicle_state.Vehicle_state'):
        mf52 = MF52()
        self.fy = mf52.Fy (Fz=self.fz,Alpha=self.alpha, Gamma=self.gamma) #TODO SCALING!!
        fx_range=[]
        for kappa in np.linspace(0,0.2,100):
            fx_range.append(mf52.Fx(Fz=self.fz, Kappa=kappa, Gamma=self.gamma))
        self.fx = max(fx_range) #TODO SCALING!!
        self.f_vec = [self.fx,self.fy,self.fz]