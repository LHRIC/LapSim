from state_models.dynamic_module import DynModel
from state_models.powertrain_module import PtnModel
from state_models.aero_module import AeroModel
from state_models.tire_state import TireState
from scipy.constants import g
import numpy as np

class VehicleState:
    def __init__(self,params:dict) -> None:
        # Pull parameter dictionary
        self.params = params
        print(self.params)
        # Initialize models
        self.dyn = DynModel(self)
        self.ptn = PtnModel(self)
        self.aero = AeroModel(self)
        # Sprung mass state
        self.roll = 0
        self.pitch = 0

    def _residuals(self):
        mz = np.linalg.matmul(self.params['inertia_tensor'],[0,0,self.psi_ddt])
        self.moments.append(mz)
        print (f'Forces: {self.forces}')
        print (f'Moments: {self.moments}')
        sum_forces = np.add.reduce(self.forces)
        sum_moments = np.add.reduce(self.moments)
        print (f'sum_Forces: {sum_forces}')
        print (f'sum_Moments: {sum_moments}')
        residuals = [*sum_forces, *sum_moments]
        return residuals

    def eval(self,v,beta,delta,eta,x_ddt,y_ddt,psi_ddt,residuals):
        # Initialize Tires
        self.fl = TireState(self)
        self.fr = TireState(self)
        self.rl = TireState(self)
        self.rr = TireState(self)
        self.forces=[]
        self.moments=[]
        self.v=v # Tangential velocity
        self.beta=beta # Body slip
        self.delta=delta # Steered angle
        self.eta=eta # 'Throttle' 
        self.x_ddt=x_ddt # Ax
        self.y_ddt=y_ddt # Ay
        self.psi_ddt=psi_ddt # Yaw accel
        # Each function updates: self.[fl,fr,rl,rr] | self.forces | self.moments
        self.dyn.static_weight(self) # Gravity effects
        self.dyn.weight_transfer(self) # Acceleration effects
        self.aero.downforce(self) # Downforce
        self.dyn.kinematic_eval(self) # Suspension travel and inclination angle
        self.ptn.torque(self) #Powertrain and braking, updates max Fx for tires
        self.dyn.steering(self) # Evaluate steering angles
        self.fl.mf52(self) # Evaluate tire forces
        self.fr.mf52(self)
        self.rl.mf52(self)
        self.rr.mf52(self)
        self.dyn.tire_forces(self) # Apply tire forces to car

        if residuals == True:
            r = self._residuals() 
            print(f'r {r}')
            return r
        

