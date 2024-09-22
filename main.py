from Simulation import Simulation
from Results import Results

results = Simulation('simulation_config.yaml')
Results.plot(results,'results_config.yaml')

