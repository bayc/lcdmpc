# -*- coding: utf-8 -*-
"""
Created on Wed Aug 14 13:35:17 2019

@author: rchintal
"""
import numpy as np

class bldg_data_driven_mdl:
    self.A = 0.9981
    self.Bu = 0.0374; # coefficients correspondig to Q_hvac - self.mean_inputs(4)
    self.Bv = [0.0038,0.0024,0.0762] #[T_out - T_room, Q_int, Q_solar] - self.mean_inputs(1:3)
    self.C = 1
    self.K = 0.9981
    self.mean_inputs = np.array([-13.5548,0.736,0.2062,0.894]); 
    self.mean_outputs = 21.8742