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
        
        s = self.kappa
        a = self.alpha
        fx0 = self.fx0
        fy0 = self.fy0
        Cs = fx0/s
        Ca = fy0/a
        
        fxy = fx0*fy0/np.sqrt(s**2*fy0**2+fx0**2*(np.tan(a)**2))
        fx_1 =  np.sqrt(s**2*Ca**2+(1-s)**2*(np.cos(a))**2*a*fx0**2)/Ca
        fy_1 =  np.sqrt((1-s)**2*(np.cos(a))**2*a*fy0**2+(np.sin(a))**2*Cs**2)/(Cs*np.cos(a))

        self.fx = fxy*fx_1
        self.fy = fxy*fy_1
        
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