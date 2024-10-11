from state_models import vehicle_state
from state_models.mf_52 import MF52 # Using Vogel_sim MF52 for now
import numpy as np
# TODO Rewrite MF52?
class TireState:
    def __init__(self,model: 'vehicle_state.VehicleState'):
        self.fz = 0.0
        self.fz_elas = 0.0
        self.z = 0.0
        self.alpha = 0.0
        self.gamma = 0.0
        self.kappa = 0.0
        self.friction_scaling_x = model.params['friction_scaling_x']
        self.friction_scaling_y = model.params['friction_scaling_y']
        self.fx_max = None
    def _comstock(self):
        # Comstock model only works in positive domains
        s = abs(self.kappa)
        a = abs(self.alpha)
        fx0 = abs(self.fx0)
        fy0 = abs(self.fy0)
        # print(f'fx0, fy0 {fx0} {fy0}')
        Cs = abs(self.c_kappa)
        Ca = abs(self.c_alpha)
        
        fxy = fx0*fy0/np.sqrt(s**2*fy0**2+fx0**2*(np.tan(a))**2)
        fx_1 = np.sqrt(s**2*Ca**2+(1-s)**2*(np.cos(a))**2*fx0**2)/Ca
        fy_1 = np.sqrt((1-s)**2*(np.cos(a))**2*fy0**2+(np.sin(a))**2*Cs**2)/(Cs*np.cos(a))
        # print(f'fxy {fxy}, fy_1 {fy_1}')
        self.fx = fxy*fx_1*np.sign(self.kappa)
        self.fy = fxy*fy_1*np.sign(self.alpha)

    def mf52(self):
        #FIXME MAJOR MAJOR WIP
        mf52 = MF52()
        self.fy0 = mf52.Fy(Fz=self.fz,Alpha=self.alpha, Gamma=self.gamma)*self.friction_scaling_y
        fx_set=[]
        kappa_set = np.linspace(0,1,100)
        for kappa in np.linspace(0,1,100):
            fx_set.append(mf52.Fx(Fz=self.fz, Kappa=kappa, Gamma=self.gamma)*self.friction_scaling_x)
        if self.fx_max is not None and max(fx_set) <= self.fx_max: # Power limited
            self.fx0 = self.fx_max
            self.kappa = np.interp(fx_set,self.fx_max,kappa_set)
        else: # Grip limited
            idx = np.argmax(fx_set)
            self.fx0 = fx_set[idx]
            self.kappa = kappa_set[idx]

        # Corner stiffnesses
        self.c_kappa = mf52.Fx(self.fz,0.05,self.gamma)/0.05
        self.c_alpha = mf52.Fy(self.fz,0.01,self.gamma)/0.01

        self._comstock()    
        
        self.f_vec = [self.fx,self.fy,self.fz] 