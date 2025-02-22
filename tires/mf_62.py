import numpy as np

class MF61:
    '''
Equations defined in:
    
Tire and Vehicle Dynamics, Third edition, Hans B. Pacejka

Chapter 4.3.2
'''

    def __init__(self,params):
        self.params = params  # Tire parameters from .tir file
        self.fz0 = 800          # Nominal wheel load -> (4.E2a)

    def fx(self,fz,kappa,gamma):
        '''
        Longitudinal Force (Pure Longitudinal Slip, alpha = 0)
        
        Neglecting turn slip, and assuming small camber values (Lamba=1)

        1's represent un-used user correction coefficents 
        and un-implemented tire pressure sensitivity

        '''
        pcx1 = self.params['PCX1']
        pdx1 = self.params['PDX1']
        pdx2 = self.params['PDX2']
        pdx3 = self.params['PDX3']
        pex1 = self.params['PEX1']
        pex2 = self.params['PEX2']
        pex3 = self.params['PEX3']
        pex4 = self.params['PEX4']
        pkx1 = self.params['PKX1']
        pkx2 = self.params['PKX2']
        pkx3 = self.params['PKX3']
        phx1 = self.params['PHX1']
        phx2 = self.params['PHX2']
        pvx1 = self.params['PVX1']
        pvx2 = self.params['PVX2']
        ppx1 = self.params['PPX1']
        ppx2 = self.params['PPX2']
        ppx3 = self.params['PPX3']
        ppx4 = self.params['PPX4']
        friction_scaling_x = self.params['friction_scaling_x']
        
        # self.xparams[19:] are R coefficents pertaining to combined slip 
        # thus not used iN pure slip calculations

        dfz = (fz-self.fz0)/(self.fz0)                              # (4.E2a)

        c_x = pcx1*1                                                # (4.E11)
        assert c_x > 0 , 'c_x must be > 0'
        mu_x = (pdx1+pdx2*dfz)*\
            (1+ppx3*1+ppx4*1**2)*(1-pdx3*gamma**2)*1                # (4.E13)
        
        # Forcing a saturation mu to prevent negative values ################
        # FIXME #############################################################
        if mu_x < 1e-2: mu_x = 1e-2
        #####################################################################

        s_hx = (phx1+phx2*dfz)*1                                    # (4.E17)
        kappa_x = kappa+s_hx                                        # (4.E10)
        e_x = (pex1+pex2*dfz+pex3*dfz**2)*\
            (1-pex4*np.sign(kappa_x))*1                             # (4.E14)
        assert e_x <= 1, 'e_x must be <= 1'
        d_x = mu_x*fz*1                                             # (4.E12)
        assert d_x > 0, 'd_x must be > 0'
        k_xk = fz*(pkx1+pkx2*dfz)*\
            np.exp(pkx3*dfz)*(1+ppx1*1+ppx2*1**2)                   # (4.E15)
        
        # Forcing a saturation slip stiffness to prevent negative values ####
        # FIXME #############################################################
        sp_sat = 12
        if (pkx1+pkx2*dfz) < sp_sat:
            k_xk = fz*(sp_sat*np.exp(1e-2*((pkx1+pkx2*dfz)-sp_sat)))*\
            np.exp(pkx3*dfz)*(1+ppx1*1+ppx2*1**2)
        #####################################################################

        b_x = k_xk/(c_x*d_x)                                        # (4.E16)
        s_vx = fz*(pvx1+pvx2*dfz)*1*1*1                             # (4.E18)

        fx0 = d_x*np.sin(c_x*np.arctan(b_x*kappa_x-e_x*(
            b_x*kappa_x-np.arctan(b_x*kappa_x))))+s_vx              # (4.E9)

        return fx0*friction_scaling_x
    
    def fy(self,fz,alpha,gamma):
        '''
        Lateral Force (Pure Side Slip, kappa = 0)
        
        Neglecting turn slip, and assuming small camber values (Lamba=1)

        1's represent un-used user correction coefficents 
        and un-implemented tire pressure sensitivity
        '''
        pcy1 = self.params['PCY1']
        pdy1 = self.params['PDY1']
        pdy2 = self.params['PDY2']
        pdy3 = self.params['PDY3']
        pey1 = self.params['PEY1']
        pey2 = self.params['PEY2']
        pey3 = self.params['PEY3']
        pey4 = self.params['PEY4']
        pey5 = self.params['PEY5']
        pky1 = self.params['PKY1']
        pky2 = self.params['PKY2']
        pky3 = self.params['PKY3']
        pky4 = self.params['PKY4']
        pky5 = self.params['PKY5']
        pky6 = self.params['PKY6']
        pky7 = self.params['PKY7']
        phy1 = self.params['PHY1']
        phy2 = self.params['PHY2']
        pvy1 = self.params['PVY1']
        pvy2 = self.params['PVY2']
        pvy3 = self.params['PVY3']
        pvy4 = self.params['PVY4']
        ppy1 = self.params['PPY1']
        ppy2 = self.params['PPY2']
        ppy3 = self.params['PPY3']
        ppy4 = self.params['PPY4']
        ppy5 = self.params['PPY5']
        friction_scaling_y = self.params['friction_scaling_y']


        dfz = (fz-self.fz0)/(self.fz0)                              # (4.E2a)
        c_y = pcy1*1                                                # (4.E21)
        assert c_y > 0 , 'c_y must be > 0'                                                          
        mu_y = (pdy1+pdy2*dfz)*(1+ppy3*1+ppy4*1**2)\
            *(1-pdy3*gamma**2)*1                                    # (4.E23)
        d_y = mu_y*fz*1                                             # (4.E22)
        k_ya = pky1*self.fz0*(1+ppy1*1)*\
        (1-pky3*np.abs(gamma))*np.sin(pky4*np.arctan(
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
        
        return fy0*friction_scaling_y