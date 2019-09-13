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

Y = power_flow_tools.extract_admittance_matrix('FourNodeSystem_EXP_Y.CSV')
Y_1 = power_flow_tools.single_phase_equivalent(Y)

np.set_printoptions(suppress=True)



# BELOW NUMBERS ARE MADE UP TO TEST CONSTRAINT-MAKING CODE. NOT REALISTIC.
v_lin = np.array([[1+0j], [1.2+0.1j], [1+0.2j], [0.9+0.3j], [1.2+0.2j]])
s_lin = np.diag(v_lin.reshape(-1)) @ np.conj(Y_1) @ np.conj(v_lin)
Y_prime, Jac_s, A, b = power_flow_tools.linear_power_flow_constraint(Y_1, s_lin, v_lin)

print(Y_1)
print('==========================================')
print(Y_prime)
print('==========================================')
print(Jac_s)
print('==========================================')
print(A)
print('==========================================')
print(b)

# Confirm that things are behaving
v_test = A @ np.concatenate([np.real(s_lin), np.imag(s_lin)]) + b
v_test = v_test[:5] + v_test[5:]*1j
# v_test matches v_lin as expected.

print('==========================================')
print(v_lin)

print('==========================================')
print(v_test)

# Conver to per unit basis
V_nom = 4160
P_nom = 5e6
Y_pu, properties_nom = power_flow_tools.convert_to_per_unit(Y, P_nom, V_nom)
