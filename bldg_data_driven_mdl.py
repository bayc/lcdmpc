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