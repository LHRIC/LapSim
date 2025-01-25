from kinematics.kinematic_model import KinematicModel

class Vehicle:
    def __init__(self):
        #TODO Logic to construct the models in different ways
        kinematic_model = KinematicModel()
        kinematic_model.from_hardpoints('hardpoints.yaml')
        return
    def evaluate(self):
        
        return