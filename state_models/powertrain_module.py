from state_models import vehicle_state
import numpy as np
from fractions import Fraction
class PtnModel:
    def __init__(self,model: 'vehicle_state.VehicleState') -> None:
        self.primary_reduction = model.params['primary_reduction']
        self.gear_ratios = {key: float(Fraction(value)) for key, value in model.params['gear_ratios'].items()}
        self.final_drive = model.params['final_drive']
        self.shiftpoint = model.params['shiftpoint']
        self.tire_radius = model.params['tire_radius']
        self.drivetrain_losses = model.params['drivetrain_losses']
        self.torque_curve_file = model.params['torque_curve_file']
        self._torque_curve()

    def torque(self,model: 'vehicle_state.VehicleState'):
        if model.eta > 0:
            self._accelerate(model)
            model.fl.free_rolling = True
            model.fr.free_rolling = True
        elif model.eta == 0:
            model.fl.free_rolling = True
            model.fr.free_rolling = True
            model.rl.free_rolling = True
            model.rr.free_rolling = True
        else:
            self._brake(model)

    def _torque_curve(self):
        # TODO No need to unpack this EVERY time, make this something that is unpacked once per vehicle sweep iteration
        self.torque_curve = np.loadtxt(self.torque_curve_file, delimiter=',', unpack=True)
    
    def _accelerate(self,model: 'vehicle_state.VehicleState'):
        wheel_rpm = 60*model.v / (2*np.pi*self.tire_radius) # Currently not taking slip ratio into account
        raw_shaft_rpm = wheel_rpm*self.primary_reduction*self.final_drive
        gear = 1
        # print(f'gear ratio {float(self.gear_ratios[1])}')
        shaft_rpm = raw_shaft_rpm*float(self.gear_ratios[1])
        while shaft_rpm > self.shiftpoint and gear < max(self.gear_ratios.keys()): #TODO Shiftpoint dictionary
            gear += 1
            shaft_rpm = raw_shaft_rpm*float(self.gear_ratios[gear])
            # print(f'gear: {gear} raw rpm {raw_shaft_rpm} adj_rpm {shaft_rpm}')
            max_torque = np.interp(shaft_rpm,self.torque_curve[0],self.torque_curve[1])
        model.rl.fx_max = max_torque*self.tire_radius*model.eta #perhaps a wrong assumption
        model.rr.fx_max = max_torque*self.tire_radius*model.eta
        # OVERRIDE FOR INFINITE PTN
        model.rl.fx_max = None
        model.rr.fx_max = None
    
    def _brake(self,model: 'vehicle_state.VehicleState'):
        model.fl.fx_max = None
        model.fr.fx_max = None
        model.rl.fx_max = None
        model.rr.fx_max = None