import numpy as np
from scipy.spatial.transform import Rotation as rot

class Hardpoint:
    def __init__(self, pos: list, fixed: bool, **kwargs):
        self.initial_pos = np.array(pos)
        self.pos = self.initial_pos
        self.fixed = fixed
        self.style = kwargs.get('style',None)
    def reset(self):
        self.pos = self.initial_pos
    def translate(self, distance: float, direction: list):
        direction = np.array(direction)
        self.pos = self.initial_pos + distance*direction

class Linkage:
    def __init__(self, point1: Hardpoint, point2: Hardpoint, **kwargs):
        self.point1 = point1
        self.point2 = point2
        self.distance = np.linalg.norm(self.point1.pos-self.point2.pos)
        self.style = kwargs.get('style',None)
    def residual(self):
        return self.distance - np.linalg.norm(self.point1.pos-self.point2.pos)

class Spring:
    def __init__(self, point1: Hardpoint, point2: Hardpoint):
        self.point1 = point1
        self.point2 = point2
    def length(self):
        return np.linalg.norm(self.point1.pos - self.point2.pos)
    def force(self) -> float:
        dynamic_length = np.linalg.norm(self.point1-self.point2)
        length_delta = self.static_length - dynamic_length
        force = -1*self.k*length_delta
        return force

class ControlledPoint:
    def __init__(self, reference: Hardpoint, axis:int, value:float):
        self.reference = reference
        self.axis = axis
        self.value = value
    def residual(self):
        return self.value - self.reference.pos[self.axis]
    
class LinearActuator:
    def __init__(self, point1: Hardpoint, point2: Hardpoint, length, **kwargs):
        self.point1 = point1
        self.point2 = point2
        self.length = length
        self.style = kwargs.get('style',None)
    def residual(self):
        real_length = np.linalg.norm(self.point1.pos - self.point2.pos)
        return real_length - self.length

class Axis:
    def __init__(self,axis_point: Hardpoint, plane_point1: Hardpoint, plane_point2: Hardpoint):
        axis2point1 = (plane_point1.pos - axis_point.pos)
        axis2point2 = (plane_point2.pos - axis_point.pos)
        self.direction = np.cross(axis2point1,axis2point2)
        self.pos = axis_point.pos

class Normal:
    def __init__(self, axis: Axis, point: Hardpoint):
        self.axis = axis
        self.point = point
    def residual(self):
        dot = np.dot(self.axis.direction, self.point.pos - self.axis.pos)
        return dot

class CoordinateSystem:
    def __init__(self, point1: Hardpoint, point2: Hardpoint, point3: Hardpoint):
        self.zero = point1.pos
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3
        self.axis1 = (self.point2.pos - self.point1.pos) / np.linalg.norm(self.point2.pos - self.point1.pos)
        self.axis2 = np.cross(self.point2.pos-self.point1.pos,self.point3.pos-self.point1.pos)/np.linalg.norm(np.cross(self.point2.pos-self.point1.pos,self.point3.pos-self.point1.pos))
        self.axis3 = np.cross(self.axis1, self.axis2)/np.linalg.norm(np.cross(self.axis1, self.axis2))
        self.axis1_initial = self.axis1
        self.axis2_initial = self.axis2
        self.axis3_initial = self.axis3
        self.matrix = np.array([self.axis1,self.axis2,self.axis3])
        self.initial_matrix = np.array([self.axis1_initial,self.axis2_initial,self.axis3_initial])
    def update(self):
        self.axis1 = (self.point2.pos - self.point1.pos) / np.linalg.norm(self.point2.pos - self.point1.pos)
        self.axis2 = np.cross(self.point2.pos-self.point1.pos,self.point3.pos-self.point1.pos)/np.linalg.norm(np.cross(self.point2.pos-self.point1.pos,self.point3.pos-self.point1.pos))
        self.axis3 = np.cross(self.axis1, self.axis2)/np.linalg.norm(np.cross(self.axis1, self.axis2))
        self.matrix = np.array([self.axis1,self.axis2,self.axis3])
    def delta_angle(self):
        delta_mat = np.linalg.inv(self.initial_matrix) @ self.matrix
        rot_obj = rot.from_matrix(delta_mat)
        euler_angles = rot_obj.as_euler('xyz')
        return euler_angles

class Rigid:
    def __init__(self, point1: Hardpoint, point2: Hardpoint, coordinate_system: CoordinateSystem):
        self.point1 = point1
        self.point2 = point2
        self.coordinate_system = coordinate_system
        self.r_vec_world = self.point1.pos - self.point2.pos
        self.r_vec = self.coordinate_system.matrix @ self.r_vec_world
    def update(self):
        self.coordinate_system.update()
        self.point1.pos = np.linalg.inv(self.coordinate_system.matrix) @ self.r_vec + self.point2.pos
    
class SuspensionCorner:
    def __init__(self, hardpoints: dict, arb_exists: bool):
        self.lower_outboard = Hardpoint(hardpoints['LO'],False)
        self.lower_inboard_fore = Hardpoint(hardpoints['LIF'],True)
        self.lower_inboard_aft = Hardpoint(hardpoints['LIA'],True)
        self.upper_outboard = Hardpoint(hardpoints['UO'],False)
        self.upper_inboard_fore = Hardpoint(hardpoints['UIF'],True)
        self.upper_inboard_aft = Hardpoint(hardpoints['UIA'],True)
        self.outboard_tie = Hardpoint(hardpoints['OT'],False)
        self.inboard_tie = Hardpoint(hardpoints['IT'],True)
        self.contact_patch = Hardpoint(hardpoints['CP'],False)
        self.wheel_center = Hardpoint(hardpoints['WC'],False)
        self.outboard_prod = Hardpoint(hardpoints['OP'],False)
        self.inboard_prod = Hardpoint(hardpoints['IP'],False)
        self.bellcrank_anchor = Hardpoint(hardpoints['BC'],True)
        self.shock_outboard = Hardpoint(hardpoints['SO'],False)
        self.shock_inboard = Hardpoint(hardpoints['SI'],True)

        self.ca_lower_aft = Linkage(self.lower_outboard,self.lower_inboard_aft)
        self.ca_lower_fore = Linkage(self.lower_outboard,self.lower_inboard_fore)
        self.ca_upper_aft = Linkage(self.upper_outboard,self.upper_inboard_aft)
        self.ca_upper_fore = Linkage(self.upper_outboard, self.upper_inboard_fore)
        self.tie_rod = Linkage(self.outboard_tie,self.inboard_tie)
        self.p_rod = Linkage(self.outboard_prod,self.inboard_prod,style="p_rod")
        self.bc_1 = Linkage(self.bellcrank_anchor,self.inboard_prod)
        self.bc_2 = Linkage(self.bellcrank_anchor,self.shock_outboard)
        self.bc_3 = Linkage(self.inboard_prod,self.shock_outboard)
        self.kpi = Linkage(self.lower_outboard,self.upper_outboard)
        self.apex1, self.apex2, self.apex3 = self.apex_generation()
        self.uns1, self.uns2  = self.unsprung_generation()

        self.shock = Spring(self.shock_outboard,self.shock_inboard)
        self.linear = LinearActuator(self.shock_outboard,self.shock_inboard,None)
        self.bc_axis = Axis(self.bellcrank_anchor, self.inboard_prod, self.shock_outboard)
        self.axis_check1 = Normal(self.bc_axis,self.inboard_prod)
        self.axis_check2 = Normal(self.bc_axis,self.shock_outboard)
        self.wheel_sys = CoordinateSystem(self.lower_outboard,self.upper_outboard,self.outboard_tie)
        self.wheel = Rigid(self.contact_patch,self.lower_outboard,self.wheel_sys)

        self.linkage_list = [self.ca_lower_aft,
                             self.ca_lower_fore,
                             self.ca_upper_aft,
                             self.ca_upper_fore,
                             self.tie_rod,
                             self.p_rod,
                             self.bc_1,
                             self.bc_2,
                             self.bc_3,
                             self.kpi,
                             self.apex1,
                             self.apex2,
                             self.apex3,
                             self.uns1,
                             self.uns2]
        
        self.point_list = [self.lower_outboard,
                            self.lower_inboard_fore,
                            self.lower_inboard_aft,
                            self.upper_outboard,
                            self.upper_inboard_fore,
                            self.upper_inboard_aft,
                            self.outboard_tie,
                            self.inboard_tie,
                            self.outboard_prod,
                            self.inboard_prod,
                            self.bellcrank_anchor,
                            self.shock_outboard,
                            self.shock_inboard]
        
        self.dependent_objects = [point for point in self.point_list if point.fixed != True]
        self.residual_objects = self.linkage_list + [self.linear, self.axis_check1, self.axis_check2]
        self.update_objects = [self.wheel]
        
    def apex_generation(self):
        lower_apex_length = np.linalg.norm(self.outboard_prod.pos - self.lower_outboard.pos)
        upper_apex_length = np.linalg.norm(self.outboard_prod.pos - self.upper_outboard.pos)
        if lower_apex_length < upper_apex_length: # Pushrod
            apex1 = Linkage(self.outboard_prod,self.lower_outboard,style="invisible")
            apex2 = Linkage(self.outboard_prod,self.lower_inboard_aft,style="invisible")
            apex3 = Linkage(self.outboard_prod,self.lower_inboard_fore,style="invisible")
        else: # Pullrod
            apex1 = Linkage(self.outboard_prod,self.upper_outboard,style="invisible")
            apex2 = Linkage(self.outboard_prod,self.upper_inboard_aft,style="invisible")
            apex3 = Linkage(self.outboard_prod,self.upper_inboard_fore,style="invisible")
        return apex1, apex2, apex3
    
    def unsprung_generation(self):
        uns1 = Linkage(self.outboard_tie,self.upper_outboard)
        uns2 = Linkage(self.outboard_tie,self.lower_outboard)
        return uns1, uns2
    