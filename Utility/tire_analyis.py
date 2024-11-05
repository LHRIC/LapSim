import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from state_models.mf_52 import MF52
from state_models.mf_61 import MF61
from state_models.tire_state import TireState
import pandas as pd

def combined_slip (alphas,kappas,fz,n):
    class psuedoVehicle:
        def __init__(self):
            self.params = {'friction_scaling_x': 0.6, 'friction_scaling_y': 0.6, 'tire_file': 'vehicles\FSAE_Defaults.tir'}
            self.eta = 0
    vehicle = psuedoVehicle()
    alpha_set = np.linspace(alphas[0],alphas[1],n)
    kappa_set = np.linspace(kappas[0],kappas[1],n)
    tire = TireState(vehicle)
    mf = MF61()
    tire.fz = fz
    fx_list = np.zeros((n,n))
    fy_list = np.zeros((n,n))
    alpha_list = np.zeros((n,n))
    kappa_list = np.zeros((n,n))
    for i, alpha in enumerate(alpha_set):
        for j, kappa in enumerate(kappa_set):
            tire.alpha = np.deg2rad(alpha)
            tire.kappa = kappa
            tire.fx0 = mf.fx(tire.fz,tire.kappa,tire.gamma)
            tire.fy0 = mf.fy(tire.fz,tire.alpha,tire.gamma)
            tire.c_kappa = mf.fx(tire.fz,0.05,tire.gamma)/0.05
            tire.c_alpha = mf.fy(tire.fz,0.02,tire.gamma)/0.02
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

def single_slip(alphas,kappas,fzs,n1,n2):
    class psuedoVehicle:
        def __init__(self):
            self.params = {'friction_scaling_x': 0.6, 'friction_scaling_y': 0.6, 'tire_file': 'vehicles\FSAE_Defaults.tir'}
            self.eta = 0
    vehicle = psuedoVehicle()
    tire = TireState(vehicle)
    mf = MF61(tire.params)
    alpha_set = np.linspace(alphas[0],alphas[1],n1)
    kappa_set = np.linspace(kappas[0],kappas[1],n1)
    fz_set = np.linspace(fzs[0],fzs[1],n2)
    fy_list = np.zeros((n2,n1))
    fx_list = np.zeros((n2,n1))
    alpha_list = np.zeros((n2,n1))
    kappa_list = np.zeros((n2,n1))

    for i, fz in enumerate(fz_set):
        for j, alpha in enumerate(alpha_set):
            tire.fz = fz
            tire.alpha=np.deg2rad(alpha)
            tire.fy0=mf.fy(tire.fz,tire.alpha,tire.gamma)
            fy_list[i,j]=tire.fy0
            alpha_list[i,j]=tire.alpha

    for i, fz in enumerate(fz_set):
        for j, kappa in enumerate(kappa_set):
            tire.fz = fz
            tire.kappa=kappa
            tire.fx0=mf.fx(tire.fz,tire.kappa,tire.gamma)
            fx_list[i,j]=tire.fx0
            kappa_list[i,j]=tire.kappa

    mf.stiffness_tracker=[]
    for i, fz in enumerate(fz_set):
        tire.fz = fz
        tire.kappa = 1
        tire.fx0=mf.fx(tire.fz,tire.kappa,tire.gamma)
    
    fig1=plt.figure()
    ax1 = fig1.add_subplot()
    ax1.set_xlabel('Slip Angle')
    ax1.set_ylabel('Fy')

    for i, fz in enumerate(fz_set):
        ax1.plot(alpha_list[i],fy_list[i],label=f'{fz}')
    ax1.legend()

    fig1=plt.figure()
    ax1 = fig1.add_subplot()
    ax1.set_xlabel('Slip Ratio')
    ax1.set_ylabel('Fx')

    for i, fz in enumerate(fz_set):
        ax1.plot(kappa_list[i],fx_list[i],label=f'{fz}')
    ax1.legend()

    k_xk_list = []
    b_x_list = []
    for i, val in enumerate(mf.stiffness_tracker):
        k_xk_list.append(val[0]/10000)
        b_x_list.append(val[1])
    fig2=plt.figure()
    ax2 = fig2.add_subplot()
    ax2.set_xlabel('Fz')
    ax2.set_ylabel('K_xk')
    ax2.plot(fz_set,k_xk_list,label='k_xk/1E4')
    ax2.plot(fz_set,b_x_list,label='bx')
    ax2.legend()
    plt.show()