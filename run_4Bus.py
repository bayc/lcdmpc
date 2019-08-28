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

print(Y)

# Conver to per unit basis
V_nom = 4160
P_nom = 5e6
Y_pu, properties_nom = power_flow_tools.convert_to_per_unit(Y, P_nom, V_nom)
