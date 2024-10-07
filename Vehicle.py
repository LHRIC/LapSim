from utility import parser
import numpy as np
import pandas as pd
from state_models.vehicle_state import Vehicle_state
from matplotlib import pyplot as plt
from scipy.optimize._root import root
from scipy.optimize import least_squares
import scipy.optimize
class Vehicle:

    def __init__(self, cfg:dict, sweep_idx):
        print('initializing vehicle object')
        parameters_file = cfg['vehicle']
        print(f'reading "{cfg['vehicle']}"')
        vehicle_cfg: dict = parser.read_yaml(parameters_file)
        # TODO auto unit conversion into metric, replace swept parameters with the corresponding sweep_idx
        kgms_params = parser.convert(vehicle_cfg['params'])
        self.params = kgms_params
        if vehicle_cfg['precalculated_envelope'] != None:
            print('Precalculated performance envelopes are not currently supported')
            self._generate(cfg) #remove after adding precalcualted performance envelope support
        else:
            self._generate(cfg)
        # self._debug()

    def _debug(self): # Temporary module to debug vehicle_state
        fl_list=[]
        fr_list=[]
        rl_list=[]
        rr_list=[]
        v,beta,delta,eta=0,0,0,0
        rng = np.linspace(0,9.81,100)
        for ay in rng:
            x=[0,ay,0]
            vehicle_state = Vehicle_state(self.params)
            vehicle_state.eval(v,beta,delta,eta,x[0],x[1],x[2],residuals=False)
            fl_list.append(vehicle_state.fl.fz)
            fr_list.append(vehicle_state.fr.fz)
            rl_list.append(vehicle_state.rl.fz)
            rr_list.append(vehicle_state.rr.fz)
        # plt.scatter(rng,fl_list)
        # plt.scatter(rng,fr_list)
        # plt.show()

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
        print('initializing vehicle_state class')
        vehicle_state = Vehicle_state(self.params)
        self.count=0
        for v in velocity_set:
            for beta in body_slip_set:
                for delta in steered_angle_set:
                    for eta in throttle_set:
                        def _solve(x):
                            print(f'{self.count} x:: {x}')
                            r = vehicle_state.eval(v,beta,delta,eta,x[0],x[1],x[2],residuals=True)
                            print(f'{self.count} RESIDUALS:: {r}')
                            self.count += 1
                            return r
                        # soln = root(_solve,x0,method='hybr')
                        soln = least_squares(_solve,x0)
                        print(soln)
                        surface_x[index]=[v,beta,delta,eta]
                        surface_y[index]=soln.x
                        x0 = soln.x
                        index+=1
        ax_list = []
        ay_list = []
        for i, row in enumerate(surface_x):
            ax_list.append(surface_y[i][0])
            ay_list.append(surface_y[i][1])
            print(f'{surface_x[i]}  ||||   {surface_y[i]}')       
        plt.scatter(ax_list,ay_list)         
        plt.show()

  
  
        

                    
