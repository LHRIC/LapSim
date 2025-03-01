class ShockModel:
    def __init__(self,k,b):
        self.k = k
        self.b = b
        return
    def force_absolute(self, x, v):
        f_d = self.force_damper(v)
        f_x = self.force_spring(x)
        return f_d + f_x
    def force_damper(self, v):
        #TODO add damper curves and stuff
        force_abs = self.b*v
        return force_abs
    def force_spring(self, x):
        force_abs = self.k*x
        return force_abs