from state.state_vector import StateVector14
from state.control_vector import ControlVector
from kinematics.kinematic_model import KinematicModel
from dynamics.shock_model import ShockModel


class Vehicle:
    def __init__(self, x0, u0):
        #TODO Logic to construct the models in different ways
        self.state_vector = StateVector14(x0)
        self.control_vector = ControlVector(u0)
        self.kinematic_model = KinematicModel()
        self.kinematic_model.from_hardpoints('parameters/hardpoints.yaml')
        self.front_shock = ShockModel(40000,0.23529411764)
        self.rear_shock = ShockModel(40000,0.23529411764)

        return
    def evaluate(self):
        self.state_vector.unpack()

        for wheel in ['fl','fr','rl','rr']:
            z_xx = eval("self.state_vector.z_" + wheel)
            kinvec = self.kinematic_model.interpolate(z_xx, self.control_vector[0])
            


        return