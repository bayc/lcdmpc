# M. Sinner 8/25/19

# Try to get opendss running on simple test case.

import opendssdirect as dss
import pandas as pd
import numpy as np

import power_flow_tools

# Start with the IEEE13bus feeder
dss.run_command('clear')
dss.run_command('Redirect tests/data/13Bus/IEEE13Nodeckt_MS.dss')
print(dss.Circuit.AllBusNames())

Y = power_flow_tools.extract_admittance_matrix('IEEE13Nodeckt_EXP_Y.CSV')

# Conver to per unit basis
V_nom = 4160
P_nom = 5e6
Y_pu, properties_nom = power_flow_tools.convert_to_per_unit(Y, P_nom, V_nom)