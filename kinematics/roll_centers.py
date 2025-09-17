import numpy as np

class Plane:
    def __init__(self, point1, point2, point3):
        self.point = point1
        self.norm_vec = np.cross(point2 - point1, point3 - point1)
        self.norm_vec = self.norm_vec/np.linalg.norm(self.norm_vec)
        self.d = np.dot(self.norm_vec, point1)

class Axis:
    def __init__(self, point, vector):
        self.point = point
        self.vector = vector

def plane_plane_intersection(plane1:Plane, plane2:Plane):
    c1 = (plane1.d - plane2.d*np.dot(plane1.norm_vec, plane2.norm_vec))\
    /(1 - np.dot(plane1.norm_vec, plane2.norm_vec)**2)
    c2 = (plane2.d - plane1.d*np.dot(plane1.norm_vec, plane2.norm_vec))\
    /(1 - np.dot(plane1.norm_vec, plane2.norm_vec)**2)
    point = c1*plane1.norm_vec + c2*plane2.norm_vec
    vector = np.cross(plane1.norm_vec, plane2.norm_vec)
    vector = vector / np.linalg.norm(vector)
    return Axis(point, vector)

def plane_axis_intersection(plane:Plane, axis:Axis):
    t = np.dot((plane.point - axis.point), plane.norm_vec)/(np.dot(axis.vector, plane.norm_vec))
    point = axis.point + t*axis.vector
    return point

def cp_roll_projection(LO, LIF, LIA, UO, UIF, UIA, CP):
    lower_plane = Plane(LO, LIF, LIA)
    upper_plane = Plane(UO, UIF, UIA)
    intersection_axis = plane_plane_intersection(lower_plane, upper_plane)
    y_plane = Plane(CP, CP + np.array([0,1,0]), CP + np.array([0,0,1]))
    instant_center = plane_axis_intersection(y_plane, intersection_axis)
    return instant_center






    
