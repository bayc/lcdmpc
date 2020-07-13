# Copyright 2020 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not 
# use this file except in compliance with the License. You may obtain a copy of 
# the License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT 
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the 
# license for the specific language governing permissions and limitations under 
# the License.

import lcdmpc as opt
import numpy as np
import matplotlib.pyplot as plt
# import numpy.linalg.matrix_power as matpow

from models.control.bldg_data_driven_mdl import bldg_data_driven_mdl
from models.simulation.bldg_sim_mdl import bldg_sim_mdl
from models.control.grid_aggregator import grid_aggregator
# from models.control.wind_mdl import wind_mdl
# from floris.floris import Floris

start_time = 750 # Start time in minutes
dt = 5          # Time-step in minutes

tmp = opt.LCDMPC(start_time, dt)

time = 240       # Length of simulation in minutes
horiz_len = 2   # Prediction horizion length
commuincation_iterations = 1
Beta = 0.5      # Convex combination parameter for control action

time_array = np.arange(start_time, (start_time + time), dt)

ms_dot = 8.0
T_sa = 12.8
T_oa = 28.
T_z = 22.794
T_e = 20.

Q_int = 8.0
Q_solar = 15.0

inputs = [ms_dot, T_sa, T_oa, T_z]

disturb1 = [6.0, 2.0, 2.0]
disturb2 = [6.0, 2.0, 2.0]

outputs1 = [1]
outputs2 = [1]

refs1 = [-4., 15.]
refs_grid = [0.]
# refs2 = [-1]

num_downstream1 = 1

bldg1_disturb_file = 'input/ROM_simulation_data.csv'

bldg1_cont = bldg_data_driven_mdl(ms_dot, T_sa, T_z, horiz_len)
# bldg2_cont = bldg_data_driven_mdl(ms_dot, T_sa, T_z, horiz_len)

bldg1_truth = bldg_sim_mdl(dt/12, ms_dot, T_sa, T_z, T_e, start_time)

grid_agg1_cont = grid_aggregator(horiz_len, num_downstream1)
grid_agg1_truth = grid_aggregator(horiz_len, num_downstream1)


### WIND MODEL ###

# input_file = 'example_input.json'

# yaw = [0.5, 0.5]
# Ct = [0.5, 0.5]
# ws = 8.
# wd = 359.

# wrefs1 = [-20000]
# wBeta = 0.0
# winputs1 = [yaw, Ct, ws, wd]
# wdisturb1 = [8., 359.]
# woutputs1 = [4.27]

# wind1 = wind_mdl(input_file, yaw, Ct, ws, wd, horiz_len)

tmp.build_subsystem(bldg1_cont, bldg1_truth, 
    inputs, outputs1, refs1, horiz_len, Beta, bldg1_disturb_file)

tmp.build_subsystem(grid_agg1_cont, grid_agg1_truth, 
    inputs, outputs1, refs_grid, horiz_len, Beta, bldg1_disturb_file)

# tmp.subsystems[0].Uconv = [ms_dot, T_sa]
# outputs = tmp.simulate_truth_model()

# print('sim model outputs: ', outputs)
# lkj

# tmp.build_subsystem(bldg2, 
#     inputs, disturb2, outputs2, refs2, horiz_len, Beta)

# tmp.build_subsystem(wind1, winputs1, wdisturb1, woutputs1, 
                    # wrefs1, horiz_len, wBeta)
# tmp.build_subsystem(wind1, winputs, woutputs2, wrefs2, horiz_len, wBeta)

# tmp.build_subsystem(A, Bu, Bv, Bd, Cy, Cz, Dyu, Dyv, Dzu, Dzv, 
#     inputs, outputs1, refs1, horiz_len, Beta, cons1)
# tmp.build_subsystem(A*2, Bu*0.2, Bv, Bd, Cy, Cz, Dyu, Dyv, Dzu, Dzv, 
#     inputs, outputs2, refs2, horiz_len, Beta, cons1)

# tmp.subsystems[0].sys_matrices()

connections = [[0, 1], [1, 0]]
# connections = [[1, 0]]

tmp.build_interconnections(interconnections=connections)
# tmp.build_interconnections(interconnections=None)

cont1 = []
out1 = []
cont2 = []
out2 = []
T_bldg = []
P_bldg = []

for i in range(int(time/dt)):

    # TODO: add Rohit's filter update
    # TODO: Need to map states to updated state (LPV like)

    tmp.relinearize_subsystem_models()

    for j in range(commuincation_iterations):
        tmp.communicate()       # Communication step

        tmp.optimize_all()

        tmp.convex_sum_cont()

        tmp.update_downstream_outputs()

        # tmp.calc_obj()

        tmp.calculate_sensitivities()

        print('==============================')
        print('communication iteration: ', j)
        print('==============================')

    tmp.calc_obj()

    tmp.update_states()

    tmp.update_subsystem_outputs()

    outputs = tmp.simulate_truth_model()
    print('outputs in main.py: ', outputs)

    T_bldg.append(outputs[0])
    P_bldg.append(outputs[1])

    tmp.update_control_filter()

    tmp.update_inputs_for_linearization()

    tmp.update_forecast_inputs()

    # cont1.append(tmp.subsystems[0].uConv[0:2*len(yaw)])
    cont1.append(tmp.subsystems[0].uConv)
    out1.append(tmp.subsystems[0].y)

    # cont2.append(tmp.subsystems[1].uOpt[0:3])
    # out2.append(tmp.subsystems[1].y)

    print('+++++++++++++++++++++++++++++')
    print('time iteration: ', i)
    print('+++++++++++++++++++++++++++++')

# print(cont1)
# print(out1)
# print(cont2)
# print(out2)

np.set_printoptions(suppress=True)

print('P_bldg: ', P_bldg)

T_lower = 21.5
T_upper = 24.5
P_lower = 0.0
P_upper = 100.0

fig, axes = plt.subplots(1, 2, figsize=(12,5))
ax1 = axes[0]
ax1.plot(time_array, np.array(T_bldg).flatten(), '-b')
ax1.plot(time_array, T_lower*np.ones(len(T_bldg)), '--k')
ax1.plot(time_array, T_upper*np.ones(len(T_bldg)), '--k')
ax1.legend(['Zone Temp', 'Temp Constraints'])
xticks = ax1.get_xticks()
ax1.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax1.set_title('Building Temperature')
ax1.set_ylabel('Temperature [C]')
ax1.set_xlabel('Time')

ax2 = axes[1]
ax2.plot(time_array, np.array(P_bldg).flatten(), '-b')
ax2.plot(time_array, [val[1] for val in tmp.subsystems[0].refs_plot[1:]], '--r')
ax2.legend(['Bldg Power', 'Power Ref'])
xticks = ax2.get_xticks()
ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax2.set_title('Building Power')
ax2.set_ylabel('Power [kw]')
ax2.set_xlabel('Time')

plt.show()
