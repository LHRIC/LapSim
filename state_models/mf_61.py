import numpy as np

class MF61:
    '''
Equations defined in:
    
Tire and Vehicle Dynamics, Third edition, Hans B. Pacejka

Chapter 4.3.2
'''

    def __init__(self,xparams,yparams):
        self.xparams = xparams  # Tire parameters for longitudinal force
        self.yparams = yparams  # Tire parameters for lateral force
        self.fz0 = 800          # Nominal wheel load -> (4.E2a)

    def fx(self,fz,kappa,gamma):
        '''
        Longitudinal Force (Pure Longitudinal Slip, alpha = 0)
        
        Neglecting turn slip, and assuming small camber values (Lamba=1)

        1's represent un-used user correction coefficents 
        and un-implemented tire pressure sensitivity

        '''
        pcx1 = self.xparams[0]
        pdx1 = self.xparams[1]
        pdx2 = self.xparams[2]
        pdx3 = self.xparams[3]
        pex1 = self.xparams[4]
        pex2 = self.xparams[5]
        pex3 = self.xparams[6]
        pex4 = self.xparams[7]
        pkx1 = self.xparams[8]
        pkx2 = self.xparams[9]
        pkx3 = self.xparams[10]
        phx1 = self.xparams[11]
        phx2 = self.xparams[12]
        pvx1 = self.xparams[13]
        pvx2 = self.xparams[14]
        ppx1 = self.xparams[15]
        ppx2 = self.xparams[16]
        ppx3 = self.xparams[17]
        ppx4 = self.xparams[18]
        
        # self.xparams[19:] are R coefficents pertaining to combined slip 
        # thus not used iN pure slip calculations

        dfz = (fz-self.fz0)/(self.fz0)                              # (4.E2a)

        c_x = pcx1*1                                                # (4.E11)
        assert c_x > 0 , 'c_x must be > 0'
        mu_x = (pdx1+pdx2*dfz)*\
            (1+ppx3*1+ppx4*1**2)*(1-pdx3*gamma**2)*1                # (4.E13)
        s_hx = (phx1+phx2*dfz)*1                                    # (4.E17)
        kappa_x = kappa+s_hx                                        # (4.E10)
        e_x = (pex1+pex2*dfz+pex3*dfz**2)*\
            (1-pex4*np.sign(kappa_x))*1                             # (4.E14)
        assert e_x <= 1, 'e_x must be <= 1'
        d_x = mu_x*fz*1                                             # (4.E12)
        assert d_x > 0, 'd_x must be > 0'
        k_xk = fz*(pkx1+pkx2*dfz)*\
            np.exp(pkx3*dfz)*(1+ppx1*1+ppx2*1**2)                   # (4.E15)
        b_x = k_xk/(c_x*d_x)                                    # (4.E16)
        s_vx = fz*(pvx1+pvx2*dfz)*1*1*1

        fx0 = d_x*np.sin(c_x*np.arctan(b_x*kappa_x-e_x*(
            b_x*kappa_x-np.arctan(b_x*kappa_x))))+s_vx              # (4.E9)

        return fx0
    
    def fy(self,fz,alpha,gamma):
        '''
        Lateral Force (Pure Side Slip, kappa = 0)
        
        Neglecting turn slip, and assuming small camber values (Lamba=1)

        1's represent un-used user correction coefficents 
        and un-implemented tire pressure sensitivity
        '''
        pcy1 = self.yparams[0]
        pdy1 = self.yparams[1]
        pdy2 = self.yparams[2]
        pdy3 = self.yparams[3]
        pey1 = self.yparams[4]
        pey2 = self.yparams[5]
        pey3 = self.yparams[6]
        pey4 = self.yparams[7]
        pey5 = self.yparams[8]
        pky1 = self.yparams[9]
        pky2 = self.yparams[10]
        pky3 = self.yparams[11]
        pky4 = self.yparams[12]
        pky5 = self.yparams[13]
        pky6 = self.yparams[14]
        pky7 = self.yparams[15]
        phy1 = self.yparams[16]
        phy2 = self.yparams[17]
        pvy1 = self.yparams[18]
        pvy2 = self.yparams[19]
        pvy3 = self.yparams[20]
        pvy4 = self.yparams[21]
        ppy1 = self.yparams[22]
        ppy2 = self.yparams[23]
        ppy3 = self.yparams[24]
        ppy4 = self.yparams[25]
        ppy5 = self.yparams[26]

        dfz = (fz-self.fz0)/(self.fz0)                              # (4.E2a)
        c_y = pcy1*1                                                # (4.E21)
        assert c_y > 0 , 'c_y must be > 0'                                                          
        mu_y = (pdy1+pdy2*dfz)*(1+ppy3*1+ppy4*1**2)\
            *(1-pdy3*gamma**2)*1                                    # (4.E23)
        d_y = mu_y*fz*1                                             # (4.E22)
        k_ya = pky1*self.fz0*(1+ppy1*1)*\
        (1-pky3*np.abs(gamma))*np.sin*(pky4*np.arctan(
            (fz/self.fz0)/((pky2+pky5*gamma**2)*(1+ppy2*1))))*1*1   # (4.E25)
        b_y = k_ya/(c_y*d_y)                                        # (4.E26)
        s_vyy = fz*(pvy3+pvy4*dfz)*gamma*1*1*1                      # (4.E28)
        s_vy = fz*(pvy1+pvy2*dfz)*1*1*1+s_vyy                       # (4.E29)
        k_yy0 = fz*(pky6+pky7*dfz)*(1+ppy5*1)*1                     # (4.E30)
        s_hy = (phy1+phy2*dfz)*1+((k_yy0*gamma-s_vyy)/(k_ya))*1     # (4.E27)
        alpha_y = alpha + s_hy                                      # (4.E20)
        e_y = (pey1+pey2*dfz)*(1+pey5*gamma**2\
        -(pey3+pey4*gamma)*np.sign(alpha_y))*1                      # (4.E24)
        assert e_y <= 1, 'e_y must be <= 1'
        fy0 = d_y*np.sin(c_y*np.arctan(
            b_y*alpha_y-e_y*(b_y*alpha_y-np.arctan(
                b_y*alpha_y))))+s_vy                                # (4.E19)
        
        return fy0