from simulation import Simulation
from results import Results
import utility.tire_analyis


# utility.tire_analyis.combined_slip((-20,20),(0,0),1503,2000)

utility.tire_analyis.single_slip((-20,20),(1000,2000),1000,15)

# results = Simulation('simulation_config.yaml')
# Results.plot(results,'results_config.yaml')

