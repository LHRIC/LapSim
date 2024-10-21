from utility import parser
import numpy as np
import pandas as pd
from state_models.vehicle_state import VehicleState
from matplotlib import pyplot as plt
from scipy.optimize._root import root
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
            vehicle_state = VehicleState(self.params)
            vehicle_state.eval(v,beta,delta,eta,x[0],x[1],x[2],residuals=False)
            fl_list.append(vehicle_state.fl.fz)
            fr_list.append(vehicle_state.fr.fz)
            rl_list.append(vehicle_state.rl.fz)
            rr_list.append(vehicle_state.rr.fz)

    def _generate(self,cfg:dict):
        print('generating response surface')
        def _param_set(param): return np.linspace(param[0],param[1],param[2])
        # TODO change for loop ordering to optimize for initial guesses
        velocity_set=_param_set(cfg['velocity_range'])
        body_slip_set=_param_set(cfg['body_slip_range'])
        steered_angle_set=_param_set(cfg['steered_angle_range'])
        yaw_rate_set=_param_set(cfg['yaw_rate_range'])
        throttle_set = _param_set(cfg['throttle_range'])
        x0 = [0,0,0]
        dim = np.prod(list(map(len,[velocity_set,yaw_rate_set,body_slip_set,steered_angle_set,throttle_set])))
        surface_x = np.zeros((dim,5))
        surface_y = np.zeros((dim,3))
        index = 0
        print('initializing vehicle_state class')
        vehicle_state = VehicleState(self.params)
        self.count=0
        for v in velocity_set:
            for psi_dt in yaw_rate_set:
                for beta in body_slip_set:
                    for delta in steered_angle_set:
                        for eta in throttle_set:
                            def _solve(x):
                                # print(f'{self.count} x:: {x}')
                                r = vehicle_state.eval(v,psi_dt,beta,delta,eta,x[0],x[1],x[2],residuals=True)
                                # print(f'{self.count} RESIDUALS:: {r}')
                                self.count += 1
                                return r
                            soln = root(_solve,x0,method='hybr',options={"xtol":1e-10})
                            # print(soln)
                            
                            # The following two arrays compose the response surface the model will be pulling from
                            surface_x[index]=[v,psi_dt,beta,delta,eta]
                            surface_y[index]=soln.x

                            # Evaluate vehicle state at solution
                            vehicle_state.eval(v,psi_dt,beta,delta,eta,soln.x[0],soln.x[1],soln.x[2],residuals=False)
                            for tire in ['fl','fr','rl','rr']:
                                print(vehicle_state.tire.f_vec)
                            # x0 = soln.x
                            index+=1
                            print(f'{index}/{dim}')
        # Plotting

        ### MMD Generation

        df_surface_x = pd.DataFrame(surface_x,columns=[
            "Velocity","Yaw_Rate","Body_Slip","Steer_Angle","Throttle"])
        df_surface_y = pd.DataFrame(surface_y,columns=[
            "Ax","Ay","Yaw_Acceleration"])
        df_surface_xy = df_surface_x.join(df_surface_y)
        df_surface_xy.to_csv("LAS.csv")

        ax_list = [idx[0]/9.81 for idx in surface_y]
        ay_list = [idx[1]/9.81 for idx in surface_y]
        psi_ddt_list = [idx[2] for idx in surface_y]
        v_list = [idx[0] for idx in surface_x]
        steer_list = [idx[3] for idx in surface_x]
        bodyslip_list = [idx[2] for idx in surface_x]
        yaw_rate_list = [idx[1] for idx in surface_x]
        eta_list = [idx[4] for idx in surface_x]
        idx_list = [i for i, val in enumerate(surface_x)]

        # ay_mat = np.reshape(ay_list,(30,30))
        # psi_ddt_mat = np.reshape(psi_ddt_list, (30,30))
        # steer_mat = np.reshape(steer_list,(30,30))
        # bodyslip_mat = np.reshape(bodyslip_list,(30,30))

        # fig3 = plt.figure()
        # ax3 = fig3.add_subplot()
        # plt.contour(ay_mat,psi_ddt_mat,steer_mat)
        # plt.contour(ay_mat,psi_ddt_mat,bodyslip_mat)

        fig0 = plt.figure()
        ax0 = fig0.add_subplot()
        sc = ax0.scatter(ay_list,ax_list, c=idx_list)
        ax0.set_xlabel('Ay')
        ax0.set_ylabel('Ax')
        plt.colorbar(sc, ax=ax0, label='psi_ddt')   

        fig05= plt.figure()
        ax05 = fig05.add_subplot()
        sc = ax05.scatter(ay_list,ax_list, c=steer_list)
        ax05.set_xlabel('Ay')
        ax05.set_ylabel('Ax')
        plt.colorbar(sc, ax=ax05, label='steer angle (rad)')   

        # fig1 = plt.figure()
        # ax1 = fig1.add_subplot()
        # sc1 = ax1.scatter(ay_list,psi_ddt_list, c=bodyslip_list)
        # ax1.set_xlabel('Ay')
        # ax1.set_ylabel('Psi_ddt')
        # plt.colorbar(sc1, ax=ax1, label='body slip (rad)')
        
        # fig2 = plt.figure()
        # ax2 = fig2.add_subplot()
        # for i, iso_steer in enumerate(steered_angle_set):
        #     plt.plot(ay_mat[i],psi_ddt_mat[i],color='red')
        #     plt.plot(ay_mat.T[i],psi_ddt_mat.T[i],color='blue')
        # plt.grid()
    
        plt.show()
        plt.show()
        plt.show()
  
        

                    
