from state.state_vector import StateVector14
from state.control_vector import ControlVector
from kinematics.kinematic_model import KinematicModel
from dynamics.shock_model import ShockModel
from tires.tire_model import TireModel


class Vehicle:
    def __init__(self, x0, u0):
        #TODO Logic to construct the models in different ways
        self.state_vector = StateVector14(x0)
        self.control_vector = ControlVector(u0)
        self.kinematic_model = KinematicModel()
        self.kinematic_model.from_hardpoints('parameters/hardpoints.yaml')
        self.front_shock = ShockModel(40000,0.23529411764)
        self.rear_shock = ShockModel(40000,0.23529411764)
        self.tire_model = TireModel()
        return
    
    def evaluate(self):
        self.state_vector.unpack()

        for i, wheel in enumerate(['fl','fr','rl','rr']):
            z_xx = eval("self.state_vector.z_" + wheel)
            z_xx_dt = eval("self.state_vector.z_" + wheel + "dt")
            w_xx_dt = eval("self.state_vector.w_" + wheel + "dt")
            kin_vec = self.kinematic_model.interpolate(z_xx, self.control_vector[0])
            motion_ratio = kin_vec[2]
            tangent_vec = kin_vec[6:8]
            wheel_pose = kin_vec[9:10]
            if wheel == "fl" or "fr":
                shock = self.front_shock
            else:
                shock = self.rear_shock
            force = shock.force_absolute(z_xx, z_xx_dt)








        return