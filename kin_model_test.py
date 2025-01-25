from kinematics.kinematic_model import KinematicModel
kinematic_model = KinematicModel()
kinematic_model.from_hardpoints('hardpoints')
kinematic_model.interpolate()