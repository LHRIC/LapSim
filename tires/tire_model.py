import numpy as np
from tires.mf_61 import MF61


class TireModel:
    def __init__(self):
        # Add .tir file parse
        self.Cfz = 200000
        self.MF = MF61(self.params)
        return
    
    def fxyz(self, tire_displacement, slip_angle, slip_ratio, inclination_angle):
        fz = self._normal_load(self, tire_displacement)
        fx0 = self.MF.fx(fz, slip_ratio, inclination_angle)
        fy0 = self.MF.fy(fz, slip_angle, inclination_angle)
        fx, fy = self._comstock(fx0, fy0, slip_angle, slip_ratio)
        return np.array([fx, fy, fz])
    
    def _normal_load(self, tire_displacement):
        fz = self.Cfz * tire_displacement
        return fz
        
    def _comstock(self, fx0, fy0, slip_angle, slip_ratio):
        # Comstock model only works in positive domains
        s = abs(slip_ratio)
        a = abs(slip_angle)
        fx0 = abs(fx0)
        fy0 = abs(fy0)
        Cs = abs(self.c_kappa)
        Ca = abs(self.c_alpha)
        # TODO Switch statement this V
        if s==0 and a==0: # Point discontinuity
            fx=0
            fy=0
        elif s==0: # Limit as defined in paper
            fx=0
            fy=fy0*np.sign(self.alpha)
        elif a==0: # Limit as defined in paper
            fx=fx0*np.sign(self.kappa)
            fy=0
        else:
            fxy = fx0*fy0/np.sqrt(s**2*fy0**2+fx0**2*(np.tan(a))**2)
            fx_1 = np.sqrt(s**2*Ca**2+(1-s)**2*(np.cos(a))**2*fx0**2)/Ca
            fy_1 = np.sqrt((1-s)**2*(np.cos(a))**2*fy0**2+(np.sin(a))**2*Cs**2)/(Cs*np.cos(a))
            fx = fxy*fx_1*np.sign(slip_ratio)
            fy = fxy*fy_1*np.sign(slip_angle)
        return fx, fy