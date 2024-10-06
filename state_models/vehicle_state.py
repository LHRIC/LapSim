from state_models.dynamic_module import Dynamic_module
from state_models.powertrain_module import Powertrain_module
from state_models.aero_module import Aero_module
from state_models.tire_state import Tire_state
from scipy.constants import g

class Vehicle_state:
    def __init__(self,params:dict) -> None:
        # Pull parameter dictionary
        self.params = params
        print(self.params)
        # Initialize models
        self.dyn = Dynamic_module(self)
        self.ptn = Powertrain_module(self)
        self.aero = Aero_module(self)
        # Sprung mass state
        self.roll = 0
        self.pitch = 0
    def _residuals(self):
        sum_fz = self.fl.fz + self.fr.fz + self.rl.fz + self.rr.fz - g*self.params['mass']
        return [0,0,sum_fz]
    
    def eval(self,v,beta,delta,eta,x_ddt,y_ddt,psi_ddt,residuals):
        # Initialize Tires
        self.fl = Tire_state()
        self.fr = Tire_state()
        self.rl = Tire_state()
        self.rr = Tire_state()
        # 
        self.v=v
        self.beta=beta
        self.delta=delta
        self.eta=eta
        self.x_ddt=x_ddt
        self.y_ddt=y_ddt
        self.psi_ddt=psi_ddt
        print(f'x_ddt: {self.x_ddt} y_ddt: {self.y_ddt}')
        # Calculate tire normal forces
        # Each function updates self.[fl,fr,rl,rr].fz
        self.dyn.static_weight(self)
        self.dyn.weight_transfer(self)
        self.aero.downforce(self)
        # Calculate suspension travel
        self.dyn.kinematic_eval(self)
        # STEERING MODULE + 
        
        # Calculate longitudinal force cutoff from powertain/braking effects
        self.ptn.torque(self)
        if residuals == True:
            return self._residuals()
        

