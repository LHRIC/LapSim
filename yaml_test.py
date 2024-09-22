from Utility.parser import read_yaml
import Vehicles
cfg = read_yaml('simulation_config.yaml')
print(cfg)
vehicle = read_yaml(cfg['vehicle'])
print(vehicle)
