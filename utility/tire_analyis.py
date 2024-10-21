import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from state_models.mf_52 import MF52
from state_models.tire_state import TireState
import pandas as pd

def combined_slip (alphas,kappas,fz,n):
    class psuedoVehicle:
        def __init__(self):
            self.params = {'friction_scaling_x': 0.6, 'friction_scaling_y': 0.6}
            self.eta = 0
    vehicle = psuedoVehicle()
    alpha_set = np.linspace(alphas[0],alphas[1],n)
    kappa_set = np.linspace(kappas[0],kappas[1],n)
    tire = TireState(vehicle)
    mf52 = MF52()
    tire.fz = fz
    fx_list = np.zeros((n,n))
    fy_list = np.zeros((n,n))
    alpha_list = np.zeros((n,n))
    kappa_list = np.zeros((n,n))
    for i, alpha in enumerate(alpha_set):
        for j, kappa in enumerate(kappa_set):
            # print(f'alpha={alpha},kappa={kappa}')
            tire.alpha = np.deg2rad(alpha)
            tire.kappa = kappa
            tire.fx0 = mf52.Fx(tire.fz,tire.kappa,tire.gamma)
            tire.fy0 = mf52.Fy(tire.fz,tire.alpha,tire.gamma)
            tire.c_kappa = mf52.Fx(tire.fz,0.05,tire.gamma)/0.05
            tire.c_alpha = mf52.Fy(tire.fz,0.02,tire.gamma)/0.02
            tire._comstock()
            fx_list[i,j] = tire.fx
            fy_list[i,j] = tire.fy
            kappa_list[i,j] = kappa
            alpha_list[i,j] = alpha
   
    # G-G Diagram
    fig0 = plt.figure()
    ax0 = fig0.add_subplot()
    sc = ax0.scatter(fy_list,fx_list,c=alpha_list)
    ax0.set_xlabel('Fy')
    ax0.set_ylabel('Fx')
    plt.colorbar(sc,ax=ax0,label='slip angle')

    # Fx surface
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.plot_surface(alpha_list, kappa_list, fx_list, cmap='viridis')
    # ax1.set_xlim(s_max,s_min)
    # ax1.set_ylim(a_min,a_max)
    ax1.set_xlabel('slip angle (deg)')
    ax1.set_ylabel('slip ratio')
    ax1.set_zlabel('Fx')
    ax1.set_title('Fx')

    # Fy surface
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.plot_surface(alpha_list, kappa_list, fy_list, cmap='viridis')
    # ax1.set_xlim(s_max,s_min)
    # ax1.set_ylim(a_min,a_max)
    ax1.set_xlabel('slip angle (deg)')
    ax1.set_ylabel('slip ratio')
    ax1.set_zlabel('Fy')
    ax1.set_title('Fy')
    plt.show()

    data = {'Slip angle': alpha_list.flatten(), 'Slip ratio': kappa_list.flatten(), 'Fx': fx_list.flatten(), 'Fy': fy_list.flatten()}
    df = pd.DataFrame(data)
    df.to_csv(f'{fz}N')

def single_slip(alphas,fzs,n1,n2):
    class psuedoVehicle:
        def __init__(self):
            self.params = {'friction_scaling_x': 0.6, 'friction_scaling_y': 0.6}
            self.eta = 0
    vehicle = psuedoVehicle()
    tire = TireState(vehicle)
    mf52 = MF52()
    alpha_set = np.linspace(alphas[0],alphas[1],n1)
    fz_set = np.linspace(fzs[0],fzs[1],n2)
    fy_list = np.zeros((n2,n1))
    alpha_list = np.zeros((n2,n1))

    for i, fz in enumerate(fz_set):
        for j, alpha in enumerate(alpha_set):
            tire.fz = fz
            tire.alpha=np.deg2rad(alpha)
            tire.fy0=mf52.Fx(tire.fz,tire.alpha,tire.gamma)
            fy_list[i,j]=tire.fy0
            alpha_list[i,j]=tire.alpha
    
    fig0=plt.figure()
    ax0 = fig0.add_subplot()
    ax0.set_xlabel('Slip Angle')
    ax0.set_ylabel('Fy')

    for i, fz in enumerate(fz_set):
        ax0.plot(alpha_list[i],fy_list[i],label=f'{fz}')
        # ax0.set_label(f'{fz}')
        # plt.annotate(f'{fz}',(alpha_list[i,n1-1],fy_list[i,n1-1]))
    ax0.legend()
    plt.show()