# M. Sinner 8/26/19

# Run 4 Bus system

import opendssdirect as dss
import pandas as pd
import numpy as np

import power_flow_tools

dss.run_command('clear')

dss.run_command('Compile tests/data/4Bus/FourNodeSystem.dss')

dss.Loads.kW(1500)

dss.run_command('Solve mode=snap')

# print(dss.Loads.First())


print(dss.Circuit.AllBusNames())

Y = power_flow_tools.extract_admittance_matrix('FourNodeSystem_EXP_Y.CSV').astype(complex)
Y_1 = power_flow_tools.single_phase_equivalent(Y)

# BELOW NUMBERS ARE MADE UP TO TEST CONSTRAINT-MAKING CODE. NOT REALISTIC.
v_lin = np.array([[1+0j], [1.2+0.1j], [1+0.2j], [0.9+0.3j], [1.2+0.2j]])
s_lin = np.array([[2+0.2j], [0+0.1j], [3+0.2j], [-1+0.5j], [-4+2j]])
A, b = power_flow_tools.linear_power_flow_constraint(Y_1, s_lin, v_lin)

# Conver to per unit basis
V_nom = 4160
P_nom = 5e6
Y_pu, properties_nom = power_flow_tools.convert_to_per_unit(Y, P_nom, V_nom)
