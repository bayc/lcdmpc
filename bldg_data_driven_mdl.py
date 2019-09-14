# -*- coding: utf-8 -*-
"""
Created on Wed Aug 14 13:35:17 2019

@author: rchintal
"""
import numpy as np
import autograd.numpy as np  # thinly-wrapped numpy
from autograd import grad    # the only autograd function you may ever need

class bldg_data_driven_mdl:
    def __init__(self, ms_dot, T_sa, T_oa, T_z):
        # fan power (kW) model coefficients
        self.a0 = 0.0029
        self.a1 = -0.0151
        self.a2 = 0.1403
        self.a3 = 0.0086
        self.hvac_cop = 3

        self.reinit(ms_dot, T_sa, T_oa, T_z)
        
    # def bldg_fan_power_model(self,ms_dot_0,ms_dot): # linearized building fan power about operating point ms_dot
    #     P0 = self.a0*(ms_dot_0**3) + self.a1*(ms_dot_0**2) + self.a2*ms_dot_0 \
    #          + self.a3
    #     fan_power = P0 + 3*self.a0*(P0**2)*(ms_dot - ms_dot_0) + \
    #                 2*self.a1*P0*(ms_dot - ms_dot_0) + self.a2*(ms_dot - ms_dot_0)
    #     return fan_power
    
    # def bldg_chiller_power_model(self,T_da_0,ms_dot_0,T_da,ms_dot,Toa):
    #     P0 = (1/self.hvac_cop)*ms_dot_0*(Toa - T_da_0)
    #     chiller_power = P0 + (1/self.hvac_cop)*(ms_dot - ms_dot_0) - \
    #                     (1/self.hvac_cop)*ms_dot_0(T_da - T_da_0)
    #     return chiller_power

    def reinit(self, ms_dot, T_sa, T_oa, T_z):
        # Model matrices
        self.A = np.array([0.8826])
        self.Bu = np.array([0.0014]) # coefficients correspondig to Q_hvac (-ve value for cooling) - self.mean_inputs(4)
        self.Bv = np.array([0.0])
        self.Bd = np.array([0.0116,0.0024,0.001]) #[T_out , Q_int, Q_solar] - self.mean_inputs(1:3)
        self.Cy = np.array([1.])
        self.Du = np.array([0.0])
        self.Dv = np.array([0.0])
        self.Dd = np.array([0.0])
        self.K = 0.8826
        self.Bu_mean_inputs = np.array([0.0])
        self.Bd_mean_inputs = np.array([19.96, 0.0, 0.0])
        self.Cy_mean_outputs = 23.56

        self.bldg_power_model(T_sa, ms_dot, T_oa, T_z)

    def bldg_fan_power(self, inputs):
        ms_dot = inputs[0]
        P_fan = self.a0*ms_dot**3 + self.a1*ms_dot**2 + self.a0*ms_dot \
              + self.a3
        return P_fan

    def bldg_chiller_power(self, inputs):
        ms_dot = inputs[0]
        T_sa = inputs[1]
        T_oa = inputs[2]
        T_z = inputs[3]
        T_ma = 0.3*T_oa + (1 - 0.3)*T_z
        P_chill = 1.005/self.hvac_cop*ms_dot*(T_ma - T_sa)
        return P_chill
        
    def bldg_power_model(self, ms_dot, T_sa, T_oa, T_z):
        grad_fan = grad(self.bldg_fan_power)
        grad_chill = grad(self.bldg_chiller_power)

        D_fan = grad_fan([ms_dot, T_sa, T_oa, T_z])
        D_chill = grad_chill([ms_dot, T_sa, T_oa, T_z])

        self.Cz = np.array([0.0])
        Dz = np.array(D_fan) + np.array(D_chill)
        self.Dzu = Dz[0:2]
        self.Dzv = np.array([0.0])
        self.Dzd = Dz[2]

        print(Dz)