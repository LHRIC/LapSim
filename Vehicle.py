from utility import parser
import numpy as np
import pandas as pd
from state_models.vehicle_state import Vehicle_state
from matplotlib import pyplot as plt
from scipy.optimize._root import root
import scipy.optimize
class Vehicle:

    def __init__(self, cfg:dict, sweep_idx):
        print('initializing vehicle object')
        parameters_file = cfg['vehicle']
        print(f'reading "{cfg['vehicle']}"')
        params: dict = parser.read_yaml(parameters_file)
        # TODO auto unit conversion into metric, replace swept parameters with the corresponding sweep_idx
        self.params = params
        if params['precalculated_envelope'] != None:
            print('Precalculated performance envelopes are not currently supported')
            self._generate(cfg) #remove after adding precalcualted performance envelope support
        else:
            self._generate(cfg)
    
    def _generate(self,cfg:dict):
        print('generating response surface')
        def _param_set(param): return np.linspace(param[0],param[1],param[2])
        # TODO change for loop ordering to optimize for initial guesses
        velocity_set=_param_set(cfg['velocity_range'])
        body_slip_set=_param_set(cfg['body_slip_range'])
        steered_angle_set=_param_set(cfg['steered_angle_range'])
        throttle_set = np.linspace(-1,1,2)
        x0 = [0,0,0]
        dim = np.prod(list(map(len,[velocity_set,body_slip_set,steered_angle_set,throttle_set])))
        surface_x = np.zeros((dim,4))
        surface_y = np.zeros((dim,3))
        index = 0
        for v in velocity_set:
            for beta in body_slip_set:
                for delta in steered_angle_set:
                    for eta in throttle_set:
                        vehicle_state = Vehicle_state(self.params)
                        def _solve(x):
                            r = vehicle_state.eval(v,beta,delta,eta,x[0],x[1],x[2],residuals=True)
                            return r
                        soln = root(_solve,x0)
                        surface_x[index]=[v,beta,delta,eta]
                        surface_y[index]=soln.x
                        x0 = soln.x
                        index+=1
                        
                        

  
  
        

                    
