# -*- coding: utf-8 -*-
"""
Created on Wed Aug 14 13:35:17 2019

@author: rchintal
"""
import numpy as np

class bldg_data_driven_mdl:
    def __init__(self):
        self.A = 0.8826
        self.Bu = 0.0014 # coefficients correspondig to Q_hvac (-ve value for cooling) - self.mean_inputs(4)
        self.Bd = [0.0116,0.0024,0.001] #[T_out , Q_int, Q_solar] - self.mean_inputs(1:3)
        self.C = 1
        self.K = 0.8826
        self.mean_inputs = np.array([19.96,0,0,0]); 
        self.mean_outputs = 23.56
        # fan power (kW) model coefficients
        self.a0 = 0.0029
        self.a1 = -0.0151
        self.a2 = 0.1403
        self.a3 = 0.0086
        self.hvac_cop = 3
        
    def bldg_fan_power_model(self,ms_dot_0,ms_dot): # linearized building fan power about operating point ms_dot
        P0 = self.a0*(ms_dot_0**3) + self.a1*(ms_dot_0**2) + self.a2*ms_dot_0 \
             + self.a3
        fan_power = P0 + 3*self.a0*(P0**2)*(ms_dot - ms_dot_0) + \
                    2*self.a1*P0*(ms_dot - ms_dot_0) + self.a2*(ms_dot - ms_dot_0)
        return fan_power
    
    def bldg_chiller_power_model(self,T_da_0,ms_dot_0,T_da,ms_dot,Toa):
        P0 = (1/self.hvac_cop)*ms_dot_0*(Toa - T_da_0)
        chiller_power = P0 + (1/self.hvac_cop)*(ms_dot - ms_dot_0) - \
                        (1/self.hvac_cop)*ms_dot_0(T_da - T_da_0)
        return chiller_power
        
        
        