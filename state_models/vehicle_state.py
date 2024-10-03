class Vehicle_state:
    def __init__(self,params:dict) -> None:
        self.params = params
        
        pass
    
    def eval(self,v,beta,delta,eta,x_ddt,y_ddt,psi_ddt,residuals):

        if residuals == True:
            return [0,0,0]
        

