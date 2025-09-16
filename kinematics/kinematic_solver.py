from kinematics.kinematic_objects import *
import numpy as np
import scipy.optimize as scipy

def kinematic_solver(dependent_objects: Hardpoint, residual_objects: Linkage | ControlledPoint, update_objects):
    initial_positions = np.array([obj.pos for obj in dependent_objects])
    positions_shape = np.shape(initial_positions)
    x0 = initial_positions.flatten()

    def _residuals(x, positions_shape, dependent_objects, residual_objects):
        positions = np.reshape(x,positions_shape)
        for i, obj in enumerate(dependent_objects):
            obj.pos = positions[i]
        residuals = np.array([obj.residual() for obj in residual_objects])
        return residuals
    
    def _objfun(x,*args):
        return _residuals(x,args[0],args[1],args[2])
        
    solution = scipy.root(_objfun,x0,tol=1e-8,args=(positions_shape,dependent_objects,residual_objects))
    soln_positions = np.reshape(solution.x,positions_shape)
    for i, obj in enumerate(dependent_objects):
        obj.pos = soln_positions[i]
    for obj in update_objects:
        obj.update()
    if solution.success == False:
        # print(solution.success,solution.nfev,solution.fun)
        pass
    return 