class ShockModel:
    def __init__(self,k,b):
        self.k = k
        self.b = b
        return
    def absolute(self,x, v):
        force_abs = 0
        force_abs += self.force_damper(v)
        force_abs += self.force_spring(x)
        return force_abs
    def force_damper(self, v):
        #TODO add damper curves and stuff
        force_abs = self.b*v
        return force_abs
    def force_spring(self, x):
        force_abs = self.k*x
        return force_abs