import numpy as np
from scipy.spatial.transform import Rotation as rot
from kinematics.kinematic_model import KinematicModel
import scipy.optimize as scipy
import sympy as sp

def integrate_roll_axis(corner: KinematicModel, steer, deg_range, deg_steps, heave_range, heave_steps):
    rad_range = np.deg2rad(deg_range)
    rad_space = np.linspace(0, rad_range, deg_steps)
    heave_space = np.linspace(heave_range[0], heave_range[1], heave_steps)
    rc_z = np.zeros((len(heave_space),len(rad_space),2))
    for i, heave in enumerate(heave_space):
        T = np.eye(4) 
        T[2,3] = heave
        shock_vals = solve_shock_vals(corner, steer, T)
        rc0 = calculate_kinematic_RCs(corner, shock_vals, steer)
        rcs = np.copy(rc0)
        rc_z[i,0] = rc0[:,2]
        for j in range(len(rad_space) - 1):
            rad_step = rad_space[j+1] - rad_space[j]
            R, T_R, T_L = np.eye(4), np.eye(4), np.eye(4)
            R[0:3,0:3] = roll_axis_Rmat(corner, shock_vals, steer, rad_step)
            T_R[0:3,3] = -rcs[0,:]
            T_L[0:3,3] = rcs[0,:]
            T = T_L @ R @ T_R @ T # Rotation about arbitrary axis
            shock_vals = solve_shock_vals(corner, steer, T)
            rcs = calculate_kinematic_RCs(corner, shock_vals, steer)
            for k, rc in enumerate(rcs):
                rc_t = (T @ np.vstack((rc.reshape(-1,1),1))).T[0,0:3]
                rcs[k] = rc_t

            rc_z[i,j+1,:] = rcs[:,2]
    return rc_z

def solve_shock_vals(corner:KinematicModel, steer, T):
    y_mirror = np.array([[1,0,0],[0,-1,0],[0,0,1]])
    interps = [corner.front_interpolator, corner.rear_interpolator]

    def obj_fun(shock_vals):
        resid = np.zeros(len(shock_vals))
        for i, interp in enumerate(interps):
            left_kin_array = corner.interpolate(shock_vals[2*i], steer, interp)
            right_kin_array = corner.interpolate(shock_vals[2*i+1], steer, interp)
            left_cp = left_kin_array[3:6].reshape(-1,1)
            left_cp = (T @ np.vstack((left_cp,1))).T[0,0:3]
            right_cp = (y_mirror @ right_kin_array[3:6]).reshape(-1,1)
            right_cp = (T @ np.vstack((right_cp,1))).T[0,0:3]
            resid[2*i] = left_cp[2]
            resid[2*i+1] = right_cp[2]
        return resid
    sol = scipy.root(obj_fun,np.zeros((4)))
    if sol.success == False:
        print(sol.message)
        print(sol.x)
    return sol.x



def roll_axis_Rmat(corner: KinematicModel, shock_vals, steer, radians):
    kinematic_RCs = calculate_kinematic_RCs(corner, shock_vals, steer)
    roll_axis = kinematic_RCs[1] - kinematic_RCs[0]
    roll_axis = roll_axis/np.linalg.norm(roll_axis)
    rot_obj = rot.from_rotvec(radians*roll_axis)
    return rot_obj.as_matrix()

def calculate_kinematic_RCs(corner: KinematicModel, shock_vals, steer):
    interps = [corner.front_interpolator, corner.rear_interpolator]
    y_mirror = np.array([[1,0,0],[0,-1,0],[0,0,1]])
    kinematic_RCs = np.zeros((2,3))

    for i, interp in enumerate(interps):
        left_kin_array = corner.interpolate(shock_vals[2*i], steer, interp)
        right_kin_array = corner.interpolate(shock_vals[2*i+1], steer, interp)
        left_IC_vec = left_kin_array[11:14] - left_kin_array[3:6]
        right_IC_vec = right_kin_array[11:14] - right_kin_array[3:6]
        kinematic_RCs[i,:] = intersection_2D(left_kin_array[3:6], left_IC_vec,
                                             y_mirror@right_kin_array[3:6], y_mirror@right_IC_vec)
    return kinematic_RCs

def intersection_2D(point1, vec1, point2, vec2):
    matrix = np.vstack((vec1[1:3], -vec2[1:3]))
    if np.linalg.cond(matrix) > 1e3:
        print(f"High matrix condition number: {np.linalg.cond(matrix)}")
        print("This suggests the instant center projections are nearly parallel")
    t_vec = np.linalg.solve(matrix, point2[1:3].reshape(-1,1) - point1[1:3].reshape(-1,1))
    intersection = point1 + t_vec[0]*vec1
    return intersection

