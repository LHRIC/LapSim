import math
import pickle
import scipy
import numpy as np

import matplotlib.pyplot as plt

class MF52:
    def __init__(self):
        self.Fx_params = np.transpose(scipy.io.loadmat("./state_models/18.0x6.0-10_R20_DriveBrakeComb.mat")["x0"])
        self.Fy_params = np.transpose(scipy.io.loadmat("./state_models/16x7.5-10_R20_Cornering.mat")["x0"])

        self.Fx_params = list(map(lambda x: x[0], self.Fx_params))
        self.Fy_params = list(map(lambda x: x[0], self.Fy_params))

        self.Fz0 = 800

    def Fx(self, Fz, Kappa, Gamma):
        lambdaFz0 = 1
        lambdaVx = 1
        lambdaMux = 1
        lambdaHx = 1
        lambdaGammax = 1
        lambdaCx = 1
        lambdaEx = 1
        lambdaKx = 1

        PCX1	=  self.Fx_params[0]
        PDX1	=  self.Fx_params[1]     
        PDX2	=  self.Fx_params[2]     
        PDX3	=  self.Fx_params[3]  	
        PEX1	=  self.Fx_params[4]  	
        PEX2	=  self.Fx_params[5]   	
        PEX3	=  self.Fx_params[6]   	
        PEX4	=  self.Fx_params[7]  	
        PKX1	=  self.Fx_params[8]     
        PKX2	=  self.Fx_params[9] 	
        PKX3	=  self.Fx_params[10]   	
        PHX1	=  self.Fx_params[11]  	
        PHX2	=  self.Fx_params[12]   	
        PVX1	=  self.Fx_params[13]  	
        PVX2	=  self.Fx_params[14]
        PEX1 = 0
        Fz0 = self.Fz0 * lambdaFz0 # Temp fix

        dfz = (Fz - Fz0) / Fz0
        SVx  = Fz * (PVX1 + PVX2 * dfz) * lambdaVx * lambdaMux
        SHx = (PHX1 + PHX2 * dfz) * lambdaHx

        KappaX = Kappa + SHx
        GammaX = Gamma * lambdaGammax
        Mux = (PDX1 + PDX2 * dfz) * (1 - PDX3 * GammaX**2) * lambdaMux

        Cx = PCX1 * lambdaCx
        Dx = Mux * Fz
        Ex = (PEX1 + PEX2 * dfz + PEX3 * dfz**2) * (1 - PEX4 * np.sign(KappaX) ) * lambdaEx
        # if Ex > 1:
        #     Ex = 1
        Kx = Fz * (PKX1 + PKX2 * dfz) * math.exp(PKX3 * dfz) * lambdaKx
        Bx = Kx/(Cx * Dx)
        Fx0 = Dx * math.sin(Cx * math.atan(Bx * KappaX - Ex * (Bx * KappaX - math.atan(Bx * KappaX)))) + SVx
        # print(f'D: {Dx} C: {Cx} B: {Bx} E: {Ex}')
        Fx = Fx0

        return Fx
    
    def Fy(self, Fz, Alpha, Gamma):
        """Return the lateral force (N) felt by a tire given a normal load (N), slip angle (rad), and camber angle (rad)"""
        
        # MF 5.2 equations require slip angle to be in degrees
        Alpha = math.degrees(Alpha)

        lambdaFz0 = 1
        lambdaVy = 1
        lambdaMuy = 1
        lambdaHy = 1
        lambdaGammay = 1
        lambdaCy = 1
        lambdaEy = 1
        lambdaKy = 1

        PCY1	=  self.Fy_params[0]
        PDY1	=  self.Fy_params[1]
        PDY2	=  self.Fy_params[2]
        PDY3	=  self.Fy_params[3]
        PEY1	=  self.Fy_params[4]
        PEY2	=  self.Fy_params[5]
        PEY3	=  self.Fy_params[6]
        PEY4	=  self.Fy_params[7]
        PEY5	=  self.Fy_params[8]
        PKY1	=  self.Fy_params[9]
        PKY2	=  self.Fy_params[10]
        PKY3	=  self.Fy_params[11]
        PKY4	=  self.Fy_params[12]
        PKY5	=  self.Fy_params[13]
        PKY6	=  self.Fy_params[14]
        PKY7	=  self.Fy_params[15]
        PHY1	=  self.Fy_params[16]
        PHY2	=  self.Fy_params[17]
        PVY1	=  self.Fy_params[18]
        PVY2	=  self.Fy_params[19]
        PVY3	=  self.Fy_params[20]
        PVY4	=  self.Fy_params[21]
        PPY1	=  self.Fy_params[22]
        PPY2	=  self.Fy_params[23]
        PPY3	=  self.Fy_params[24] 
        PPY4	=  self.Fy_params[25]
        PPY5	=  self.Fy_params[26]

        Fz0 = self.Fz0 * lambdaFz0

        dfz = (Fz - Fz0) / Fz0
        GammaY = Gamma * lambdaGammay

        SVy = Fz * ((PVY1 + PVY2 * dfz) * lambdaVy + (PVY3 + PVY4 * dfz) * GammaY) * lambdaMuy
        #SHy = (PHY1 + PHY2 * dfz) * lambdaHy + PHY3 * GammaY
        SHy = (PHY1 + PHY2 * dfz) * lambdaHy

        AlphaY = Alpha + SHy
        Muy = (PDY1 + PDY2 * dfz) * (1 - PDY3 * GammaY ** 2) * lambdaMuy

        Cy = PCY1 * lambdaCy
        Dy = Muy * Fz
        Ey = (PEY1 + PEY2 * dfz) * (1 - (PEY3 + PEY4 * GammaY) * np.sign(AlphaY)) * lambdaEy
        Ky = PKY1 * Fz0 * math.sin(2 * math.atan(Fz / (PKY2 * Fz0 * lambdaFz0))) * (1 - PKY3 * abs(GammaY)) * lambdaFz0 * lambdaKy
        By = Ky / (Cy * Dy)

        Fy0 = Dy * math.sin(Cy * math.atan(By * AlphaY - Ey * (By * AlphaY - math.atan(By * AlphaY)))) + SVy

        Fy = Fy0

        return Fy

        

if __name__ == "__main__":
    tm = MF52()

    Fx = []
    for i in np.linspace(-0.23, 0.23):
        Fx.append(tm.Fy(800, i, 0))

    fig, ax = plt.subplots()
    ax.plot(np.linspace(-0.23, 0.23), Fx)
    plt.show()

