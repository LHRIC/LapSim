from utility.parser import read_yaml
from Vehicle import Vehicle
from Path import Path

class Simulation:

    def __init__(self,config_file):
        self.cfg = read_yaml(config_file)
        self.run_types: dict = {'single_run': self.single_run}
        self.evaluation_modes: dict = {'endurance': self.endurance}
        try:
            self.run_types[self.cfg['simulation_type']]()
        except KeyError:
            print(f'"{self.cfg['simulation_type']}" is not a valid run type, check simulation_config.yaml')

## Run Types

    def single_run(self):
        print('creating vehicle object instance')
        vehicle = Vehicle(self.cfg,sweep_idx=None)
        try:
            self.evaluation_modes[self.cfg['evaluation_mode']](vehicle)
        except KeyError:
            print(f'"{self.cfg['evaluation_mode']}" is not a valid evaluation mode, check simulation_config.yaml')
    
    def sweep():
        return 

## Evaluation Modes

    def competition():
        return
    
    def endurance(self, vehicle: Vehicle):
        path = Path(self.cfg['paths']['endurance'])
        return
    
    def autocross():
        return
    
    def skipad():
        return
    
    def accel():
        return

## Internal

    def traverse():
        return
    