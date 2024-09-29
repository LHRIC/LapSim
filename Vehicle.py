from utility import parser
import numpy as np

class Vehicle:
    def __init__(self, cfg:dict, sweep_idx):
        print('initializing vehicle object')
        config_file = cfg['vehicle']
        print(f'reading "{cfg['vehicle']}"')
        params: dict = parser.read_yaml(config_file)
        # TODO auto unit conversion into metric, replace swept parameters with the corresponding sweep_idx
        self.params = params
        if params['precalculated_envelope'] != None:
            print('Precalculated performance envelopes are not currently supported')
            self._generate(cfg)
        else:
            self._generate(cfg)
    def _generate(self,cfg:dict):
        mesh = cfg['envelope_mesh']
        


        pass