from state_models import vehicle_state
from state_models.mf_52 import MF52 # Using Vogel_sim MF52 for now
from state_models.mf_61 import MF61
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar, root_scalar
from scipy.io import loadmat
from Utility.parser import parse_tir


# TODO Rewrite MF52?
class TireState:
    def __init__(self,model: 'vehicle_state.VehicleState'):
        self.params = parse_tir(model.params['tire_file'])
        self.mf = MF61(self.xparams,self.yparams)
        self.fz = 0.0
        self.fz_elas = 0.0
        self.z = 0.0
        self.alpha = 0.0
        self.gamma = 0.0
        self.kappa = 0.0
        self.friction_scaling_x = model.params['friction_scaling_x']
        self.friction_scaling_y = model.params['friction_scaling_y']
        self.fx_max = None
        self.free_rolling = False
        self.dir = np.sign(model.eta)

    def _comstock(self):
        # Comstock model only works in positive domains
        s = abs(self.kappa)
        a = abs(self.alpha)
        fx0 = abs(self.fx0)
        fy0 = abs(self.fy0)
        Cs = abs(self.c_kappa)
        Ca = abs(self.c_alpha)
        # TODO Switch statement this V
        if s==0 and a==0: # Point discontinuity
            self.fx=0
            self.fy=0
        elif s==0: # Limit as defined in paper
            self.fx=0
            self.fy=fy0*np.sign(self.alpha)
        elif a==0: # Limit as defined in paper
            self.fx=fx0*np.sign(self.kappa)
            self.fy=0
        else:
            fxy = fx0*fy0/np.sqrt(s**2*fy0**2+fx0**2*(np.tan(a))**2)
            fx_1 = np.sqrt(s**2*Ca**2+(1-s)**2*(np.cos(a))**2*fx0**2)/Ca
            fy_1 = np.sqrt((1-s)**2*(np.cos(a))**2*fy0**2+(np.sin(a))**2*Cs**2)/(Cs*np.cos(a))
            # print(f'fxy {fxy}, fy_1 {fy_1}')
            self.fx = fxy*fx_1*np.sign(self.kappa)
            self.fy = fxy*fy_1*np.sign(self.alpha)
        # print(f'fx {self.fx} fy {self.fy} fz {self.fz}')
    
    def eval(self):
        
        return

    def mf52(self):
        mf52 = MF52()
        if self.fz == 0:
            self.f_vec = [0,0,0]
            return
        self.fy0 = mf52.Fy(Fz=self.fz,Alpha=self.alpha, Gamma=self.gamma)*self.friction_scaling_y
        fx_set=[]
        # TODO Optimize this

        if self.free_rolling == True:
            self.kappa = 0.0
            self.fx0 = 0.0 # TODO add rolling resistance

        else:

            def _minum_attempt(x):
                return -self.dir*mf52.Fx(Fz=self.fz,Kappa=x,Gamma=self.gamma)
            def _root_attempt(x):
                return self.fx_max - self.dir*mf52.Fx(Fz=self.fz,Kappa=x,Gamma=self.gamma)*self.friction_scaling_x
            
            kappa_max = minimize_scalar(_minum_attempt,bounds=(0,1) if self.dir==1 else (-1,0)).x
            fx_grip_max = mf52.Fx(Fz=self.fz,Kappa=kappa_max,Gamma=self.gamma)*self.friction_scaling_x

            # kappa_set = np.linspace(0,1*self.dir,1000)
            # for kappa in kappa_set:
            #     fx_set.append(mf52.Fx(Fz=self.fz, Kappa=kappa, Gamma=self.gamma)*self.friction_scaling_x)

            if self.fx_max is not None and np.abs(fx_grip_max) >= np.abs(self.fx_max): # Power/Brake limited
                self.fx0 = self.fx_max
                self.kappa = root_scalar(_root_attempt,x0=0)

            else: # Grip limited
                self.kappa = kappa_max
                self.fx0 = fx_grip_max
            # plt.plot(kappa_set,fx_set)
            # plt.scatter(kappa_set[idx],fx_set[idx])

        # Corner stiffnesses
        self.c_kappa = mf52.Fx(self.fz,0.05,self.gamma)/0.05
        self.c_alpha = mf52.Fy(self.fz,0.01,self.gamma)/0.01
        # print(f'fx0 {self.fx0} fy0 {self.fy0}')
        self._comstock()    
        
        self.f_vec = np.array([self.fx,self.fy,self.fz] )