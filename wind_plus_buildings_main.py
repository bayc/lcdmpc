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
import pandas as pd
# import numpy.linalg.matrix_power as matpow

from models.control.bldg_data_driven_mdl_med import bldg_data_driven_mdl_med
from models.control.bldg_grid_agg_data_driven_mdl_large import bldg_grid_agg_data_driven_mdl_large
from models.control.bldg_grid_agg_data_driven_mdl_increase import bldg_grid_agg_data_driven_mdl_increase
from models.control.bldg_grid_agg_data_driven_mdl_increase_modS import bldg_grid_agg_data_driven_mdl_increase_modS
from models.control.bldg_grid_agg_data_driven_mdl_increase_small import bldg_grid_agg_data_driven_mdl_increase_small
from models.simulation.bldg_sim_mdl import bldg_sim_mdl
from models.simulation.bldg_sim_mdl_doe_med import bldg_sim_mdl_doe_med
from models.simulation.bldg_sim_mdl_increase import bldg_sim_mdl_increase
from models.simulation.bldg_sim_mdl_increase_small import bldg_sim_mdl_increase_small
from models.control.grid_aggregator import grid_aggregator
# from models.control.wind_mdl import wind_mdl
# from floris.floris import Floris

start_time = 700 # Start time in minutes
dt = 1          # Time-step in minutes

tmp = opt.LCDMPC(start_time, dt)

time = 5       # Length of simulation in minutes
horiz_len = 2   # Prediction horizion length
commuincation_iterations = 15
Beta = 0.0      # Convex combination parameter for control action

time_array = np.arange(start_time, (start_time + time), dt)

bldg1_disturb_file = 'input/ROM_simulation_data_interp.csv'
bldg1_small_disturb_file = 'input/ROM_simulation_data_small_office.csv'

num_buildings_large = 3
num_buildings_medium = 0
num_buildings_small = 0
num_buildings_total = num_buildings_large + num_buildings_medium + num_buildings_small

ms_dot_large = 8.0
T_sa_large = 12.8
T_oa_large = 28.
T_z_large = 22.794
T_e_large = 20.

ms_dot_medium = 4.0
T_sa_medium = 12.8
T_oa_medium = 28.95
T_z_medium = 24.33
T_e_medium = 20.

ms_dot_small = 1.0
T_sa_small = 12.8
T_oa_small = 28.
T_z_small = 24.3313
T_e_small = 24.

Q_int = 8.0
Q_solar = 15.0

inputs = [ms_dot_large, T_sa_large, T_oa_large, T_z_large]
inputs_large = [ms_dot_large, T_sa_large, T_oa_large, T_z_large]
inputs_medium = [ms_dot_medium, T_sa_medium, T_oa_medium, T_z_medium]
inputs_small = [ms_dot_small, T_sa_small, T_oa_small, T_z_small]

disturb1 = [6.0, 2.0, 2.0]
disturb2 = [6.0, 2.0, 2.0]

outputs1 = [1]
outputs2 = [1]

refs1 = [[0.2], [20.], [0.0]]
# refs_grid = [[40.], [0.], [0.]]

disturbance_data = pd.read_csv(bldg1_disturb_file)
Toa_horiz = disturbance_data.iloc[start_time: start_time + int(time/dt) + horiz_len]['T_outside'].values
Toa_horiz_normed = Toa_horiz/Toa_horiz[0]

np.random.seed(1)
grid_agg_ref = np.random.normal(6*num_buildings_small + 30*num_buildings_medium + 60*num_buildings_large, 5.0, int(time/dt) + horiz_len)*Toa_horiz_normed

refs_grid_total = pd.DataFrame()
for i in range(int(time/dt) + horiz_len):
    refs_grid_total = refs_grid_total.append(
        {
            'time': start_time + i,
            'grid_ref': [[grid_agg_ref[i]]] + [[0.] for i in range(num_buildings_total)]
        },
        ignore_index=True
    )

bldg_optoptions = {
    'Major feasibility tolerance': 1e-4,
    'Print file': 'SNOPT_bldg_print.out',
    'Summary file': 'SNOPT_bldg_summary.out',
}
grid_optoptions = {
    'Major feasibility tolerance': 1e0,
    'Print file': 'SNOPT_grid_print.out',
    'Summary file': 'SNOPT_grid_summary.out',
}

num_downstream1 = num_buildings_total

building_control_models = []
building_truth_models = []

# load_scale = np.random.normal(2.0, 0.2, num_buildings_total)
factor = 1

Qint_scale = np.random.normal(1.3, 0.2, num_buildings_total).tolist()
Qsol_scale = np.random.normal(1.5, 0.5, num_buildings_total).tolist()

Qint_offset = [0.0]*num_buildings_total
Qsol_offset = [0.0]*num_buildings_total

Qint_std = [1.0]*num_buildings_total
Qsol_std = [1.0]*num_buildings_total

energy_red_weight = [0.0]*num_buildings_total

for i in range(num_buildings_large):
    building_control_models.append(
        bldg_grid_agg_data_driven_mdl_increase(
            ms_dot_large, T_sa_large, T_z_large, horiz_len, energy_red_weight[i],
            Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
            Qint_offset[i], Qsol_offset[i]
        )
    )
    building_truth_models.append(
        bldg_sim_mdl_increase(dt/60, ms_dot_large, T_sa_large, T_z_large, T_e_large, start_time,
        Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
        Qint_offset[i], Qsol_offset[i])
    )

for i in range(num_buildings_medium):
    building_control_models.append(
        bldg_data_driven_mdl_doe_med(
            ms_dot_medium, T_sa_medium, T_z_medium, horiz_len, energy_red_weight[i],
            Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
            Qint_offset[i], Qsol_offset[i]
        )
    )
    building_truth_models.append(
        bldg_sim_mdl_doe_med(dt/60, ms_dot_medium, T_sa_medium, T_z_medium, T_e_medium, start_time,
        Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
        Qint_offset[i], Qsol_offset[i])
    )

for i in range(num_buildings_small):
    building_control_models.append(
        bldg_grid_agg_data_driven_mdl_increase_small(
            ms_dot_small, T_sa_small, T_z_small, horiz_len, energy_red_weight[i],
            Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
            Qint_offset[i], Qsol_offset[i]
        )
    )
    building_truth_models.append(
        bldg_sim_mdl_increase_small(dt/12, ms_dot_small, T_sa_small, T_z_small, T_e_small, start_time,
        Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
        Qint_offset[i], Qsol_offset[i])
    )

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

tmp.build_subsystem(0, grid_agg1_cont, grid_agg1_truth, 
    inputs, outputs1, horiz_len, Beta, bldg1_disturb_file, refs_total=refs_grid_total,
    optOptions=grid_optoptions)

# tmp.build_subsystem(0, grid_agg1_cont, grid_agg1_truth, 
#     inputs, outputs1, horiz_len, Beta, bldg1_disturb_file, refs=refs_grid,
#     optOptions=grid_optoptions)

for i in range(num_buildings_large):
    print('i large: ', i+1)
    tmp.build_subsystem(i+1, building_control_models[i], building_truth_models[i],
    inputs_large, outputs1, horiz_len, Beta, bldg1_disturb_file, refs=refs1,
    optOptions=bldg_optoptions)

for i in range(num_buildings_medium):
    print('i medium: ', i+1)
    tmp.build_subsystem(i+1, building_control_models[i], building_truth_models[i],
    inputs_medium, outputs1, horiz_len, Beta, bldg1_small_disturb_file, refs=refs1,
    optOptions=bldg_optoptions)

for i in range(num_buildings_small):
    print('i small: ', i+1+num_buildings_large)
    tmp.build_subsystem(i+1+num_buildings_large, building_control_models[i], building_truth_models[i],
    inputs_small, outputs1, horiz_len, Beta, bldg1_small_disturb_file, refs=refs1,
    optOptions=bldg_optoptions)

# tmp.build_subsystem(1, bldg1_cont, bldg1_truth, 
#     inputs, outputs1, refs1, horiz_len, Beta, bldg1_disturb_file,
#     optOptions=bldg_optoptions)

# tmp.build_subsystem(2, bldg2_cont, bldg2_truth, 
#     inputs, outputs1, refs1, horiz_len, Beta, bldg1_disturb_file,
#     optOptions=bldg_optoptions)

# lkj
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

# connections = [[0, 1], [1, 0], [0, 2], [2, 0]]
connections = [[0, i+1] for i in range(num_buildings_total)] + [[i+1, 0] for i in range(num_buildings_total)]
# connections = [[1, 0]]

tmp.build_interconnections(interconnections=connections)
# tmp.build_interconnections(interconnections=None)

cont1 = []
out1 = []
cont2 = []
out2 = []
T_bldg1 = []
P_bldg1 = []
T_bldg2 = []
P_bldg2 = []
P_ref = []
outputs_all = []
controls_all = []
gamma_all = []

for i in range(int(time/dt)):

    print('+++++++++++++++++++++++++++++')
    print('time iteration: ', i)
    print('+++++++++++++++++++++++++++++')

    # TODO: add Rohit's filter update
    # TODO: Need to map states to updated state (LPV like)

    tmp.relinearize_subsystem_models()

    gamma_comm = []
    for j in range(commuincation_iterations):
        tmp.communicate()       # Communication step

        tmp.optimize_all()

        tmp.convex_sum_cont()

        tmp.update_downstream_outputs()

        tmp.calc_obj()

        gamma_comm.append(tmp.calculate_sensitivities())
        

        # print('==============================')
        print('communication iteration: ', j)
        # print('==============================')

    gamma_all.append(gamma_comm)

    tmp.calc_obj()

    tmp.update_states()

    tmp.update_subsystem_outputs()

    outputs = tmp.simulate_truth_model()
    outputs_all.append(outputs)
    # print('outputs in main.py: ', outputs)

    # if i == 0:
    # T_bldg1.append(outputs[1][0])
    # P_bldg1.append(outputs[1][1])

    # T_bldg2.append(outputs[2][0])
    # P_bldg2.append(outputs[2][1])
    # if i == 1:
    #     P_ref.append(outputs)

    tmp.update_control_filter()

    tmp.update_inputs_for_linearization()

    tmp.update_forecast_inputs()

    controls_all.append([[subsys.uConv] for subsys in tmp.subsystems])

    # cont1.append(tmp.subsystems[0].uConv[0:2*len(yaw)])
    # cont1.append(tmp.subsystems[1].uConv)
    # out1.append(tmp.subsystems[1].y)

    # cont2.append(tmp.subsystems[2].uConv)
    # out2.append(tmp.subsystems[2].y)


# print('building1 control: ', cont1)
# print('building1 output: ', out1)
# print('building2 control: ', cont2)
# print('building2 output: ', out2)

np.set_printoptions(suppress=True)

# print(controls_all)
# llkj

# print('all outputs: ', [val[0] for val in outputs_all])
# print('all gamma: ', gamma_all)
print('gamma[0]: ', gamma_all[0])
print('gamma[0][0]: ', gamma_all[0][0])
# print('gamma[0][1]: ', gamma_all[0][1])
# print('gamma[0][0][1]: ', gamma_all[0][0][1:])
# print('gamma[0][0][0][0][0]: ', gamma_all[0][0][0][0][0])
# print('gamma[0][0][0][1][0]: ', gamma_all[0][0][0][1][0])
# print('gamma[0][0][0][2][0]: ', gamma_all[0][0][0][2][0])
# print('gamma[0][0][0][3][0]: ', gamma_all[0][0][0][3][0])
# print('gamma[0][1][0][0][0]: ', gamma_all[0][1][0][0][0])
# print('gamma[1][0][0][0][0]: ', gamma_all[1][0][0][0][0])
# print('gamma[1][1][0][0][0]: ', gamma_all[1][1][0][0][0])
# print('gamma[2][0][0][0][0]: ', gamma_all[2][0][0][0][0])
# print('gamma[2][1][0][0][0]: ', gamma_all[2][1][0][0][0])
# print('gamma[0][:][0][0][0]: ', gamma_all[0][:][0][0][0])
# print([val[:-1][0][0] for val in gamma_all[0]])
# print([val[:-1][0][0] for val in gamma_all[1]])

grid_gamma = []
bldg1 = []
bldg2 = []
bldg3 = []
bldg4 = []
bldg5 = []
bldg6 = []
bldg7 = []
bldg8 = []
bldg9 = []
bldg10 = []
bldg11 = []
bldg12 = []
bldg13 = []
bldg14 = []
bldg15 = []
for i in range(commuincation_iterations):
    grid_gamma.append(gamma_all[-1][i][0][0][0])
    bldg1.append(gamma_all[-1][i][1][0][0])
    bldg2.append(gamma_all[-1][i][2][0][0])
    bldg3.append(gamma_all[-1][i][3][0][0])
    # bldg4.append(gamma_all[-1][i][4][0][0])
    # bldg5.append(gamma_all[-1][i][5][0][0])
    # bldg6.append(gamma_all[-1][i][6][0][0])
    # bldg7.append(gamma_all[-1][i][7][0][0])
    # bldg8.append(gamma_all[-1][i][8][0][0])
    # bldg9.append(gamma_all[-1][i][9][0][0])
    # bldg10.append(gamma_all[-1][i][10][0][0])
    # bldg11.append(gamma_all[-1][i][11][0][0])
    # bldg12.append(gamma_all[-1][i][12][0][0])
    # bldg13.append(gamma_all[-1][i][13][0][0])
    # bldg14.append(gamma_all[-1][i][14][0][0])
    # bldg15.append(gamma_all[-1][i][15][0][0])

scale = 1

plt.figure()
plt.plot(grid_gamma)
plt.plot(np.array(bldg1)*scale)
plt.plot(np.array(bldg2)*scale)
plt.plot(np.array(bldg3)*scale)
# plt.plot(np.array(bldg4)*scale)
# plt.plot(np.array(bldg5)*scale)
# plt.plot(np.array(bldg6)*scale)
# plt.plot(np.array(bldg7)*scale)
# plt.plot(np.array(bldg8)*scale)
# plt.plot(np.array(bldg9)*scale)
plt.legend(('grid',
    'bldg1',
    'bldg2',
    'bldg3',
    'bldg4',
    'bldg5',
    'bldg6',
    'bldg7',
    'bldg8',
    'bldg9',
))

# print('all outputs: ', outputs_all[0])
# print('temps: ', np.shape(outputs_all))
# print('temps: ', np.array(outputs_all).flatten())
# print('new: ')

plot_temps = []
for i in range(num_buildings_total):
    plot_temps.append([val[i+1:][0][0][0][0] for val in outputs_all])

plot_bldg_powers = []
for i in range(num_buildings_total):
    plot_bldg_powers.append([val[i+1:][0][1][0][0] for val in outputs_all])

# plot_bldg_ref_powers = []
# for i in range(num_buildings_total):
#     plot_bldg_ref_powers.append([val[0][0][i][0][0] for val in outputs_all])
# print(plot_bldg_ref_powers)

# print('plot_bldg_powers: ', plot_bldg_powers)

# print('plot_temps: ', plot_temps)
# print('P_bldg: ', P_bldg)

total_power = []
grid_prefs_total = []
for i in range(len(outputs_all)):
    # sum_of_powers = outputs_all[i][1][1] + outputs_all[i][2][1]
    # print('powers************: ', outputs_all[i][1::][1])
    sum_of_powers = np.sum([val[1] for val in outputs_all[i][1:]])
    # sum_of_grid_prefs = outputs_all[i][0][0] + outputs_all[i][0][1]
    sum_of_grid_prefs = np.sum(outputs_all[i][0][:])
    total_power.append(sum_of_powers)
    grid_prefs_total.append(sum_of_grid_prefs)



T_lower = 21.5
T_upper = 24.5
P_lower = 0.0
P_upper = 100.0

fig, axes = plt.subplots(2, 2, figsize=(16,10))

dt_num_offset = 4

disturbance_data = pd.read_csv(bldg1_disturb_file)

ax2 = axes[0, 0]
bldg_temp_legend = []
for i in range(num_buildings_total):
    ax2.plot(time_array[dt_num_offset:], np.array(plot_temps[i]).flatten()[dt_num_offset:])
    bldg_temp_legend.append('Bldg ' + str(i))
ax2.plot(
    time_array[dt_num_offset:],
    disturbance_data.iloc[start_time + dt_num_offset: start_time + int(time/dt)]['T_outside'].values, color='gray'
)
ax2.plot(time_array[dt_num_offset:], T_lower*np.ones(len(time_array))[dt_num_offset:], '--k')
ax2.plot(time_array[dt_num_offset:], T_upper*np.ones(len(time_array))[dt_num_offset:], '--k')
ax2.legend(bldg_temp_legend + ['Outdoor Air Temp'])
xticks = ax2.get_xticks()
ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax2.set_title('Building Temperatures')
ax2.set_ylabel('Temperature [C]')
ax2.set_xlabel('Time')
# ax2.set_xlim(start_time + dt*dt_num_offset, start_time + time - dt)

ax2 = axes[1, 0]
bldg_power_legend = []
for i in range(num_buildings_total):
    ax2.plot(time_array[dt_num_offset:], np.array(plot_bldg_powers[i]).flatten()[dt_num_offset:])
    bldg_power_legend.append('Bldg ' + str(i))
ax2.legend(bldg_power_legend)
xticks = ax2.get_xticks()
ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax2.set_title('Building Powers')
ax2.set_ylabel('Power [kW]')
ax2.set_xlabel('Time')
# ax2.set_xlim(start_time + dt*dt_num_offset, start_time + time - dt)

# ax1 = axes[0, 0]
# ax1.plot(time_array, np.array(T_bldg1).flatten(), '-b')
# ax1.plot(time_array, T_lower*np.ones(len(T_bldg1)), '--k')
# ax1.plot(time_array, T_upper*np.ones(len(T_bldg1)), '--k')
# ax1.legend(['Zone Temp', 'Temp Constraints'])
# xticks = ax1.get_xticks()
# ax1.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
# ax1.set_title('Building Temperature')
# ax1.set_ylabel('Temperature [C]')
# ax1.set_xlabel('Time')

# ax2 = axes[0, 1]
# ax2.plot(time_array, np.array(P_bldg1).flatten(), '-b')
# ax2.plot(time_array, [val[1] for val in tmp.subsystems[0].refs_plot[1:]], '--r')
# ax2.legend(['Bldg Power', 'Power Ref'])
# xticks = ax2.get_xticks()
# ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
# ax2.set_title('Building Power')
# ax2.set_ylabel('Power [kw]')
# ax2.set_xlabel('Time')

# ax1 = axes[1, 0]
# ax1.plot(time_array, np.array(T_bldg2).flatten(), '-b')
# ax1.plot(time_array, T_lower*np.ones(len(T_bldg2)), '--k')
# ax1.plot(time_array, T_upper*np.ones(len(T_bldg2)), '--k')
# ax1.legend(['Zone Temp', 'Temp Constraints'])
# xticks = ax1.get_xticks()
# ax1.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
# ax1.set_title('Building Temperature')
# ax1.set_ylabel('Temperature [C]')
# ax1.set_xlabel('Time')

# ax2 = axes[1, 1]
# ax2.plot(time_array, np.array(P_bldg2).flatten(), '-b')
# ax2.plot(time_array, [val[1] for val in tmp.subsystems[1].refs_plot[1:]], '--r')
# ax2.legend(['Bldg Power', 'Power Ref'])
# xticks = ax2.get_xticks()
# ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
# ax2.set_title('Building Power')
# ax2.set_ylabel('Power [kw]')
# ax2.set_xlabel('Time')

ax4 = axes[0, 1]
ax4.plot(time_array[dt_num_offset:], np.array(total_power).flatten()[dt_num_offset:], '-b')
ax4.plot(time_array[dt_num_offset:], np.array(grid_prefs_total).flatten()[dt_num_offset:], '-r')
ax4.plot(time_array[dt_num_offset:], grid_agg_ref[dt_num_offset-1:-horiz_len-1], '-.k')
ax4.legend(['Bldg Power', 'Power Ref Stpts', 'Grid Power Ref'])
xticks = ax4.get_xticks()
ax4.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax4.set_title('Total Power')
ax4.set_ylabel('Power [kw]')
ax4.set_xlabel('Time')
# ax4.set_xlim(start_time + dt*dt_num_offset, start_time + time - dt)
# ax4.set_ylim(20, 60)

ax4 = axes[1, 1]
ax4.plot(time_array[dt_num_offset:], np.array((np.array(grid_prefs_total) - np.array(total_power))/np.array(grid_prefs_total)*100).flatten()[dt_num_offset:], '-r')
# ax4.plot(time_array[dt_num_offset:], np.array((np.array(grid_agg_ref[-1:-horiz_len-1]) - np.array(total_power))/np.array(grid_agg_ref[dt_num_offset-1:-horiz_len-1])*100).flatten()[dt_num_offset:], '-b')
# ax4.plot(time_array, np.array(grid_prefs_total).flatten(), '-r')
# ax4.legend(['Bldg Power', 'Grid Power Ref'])
xticks = ax4.get_xticks()
ax4.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax4.set_title('Power Error')
ax4.set_ylabel('Error [%]')
ax4.set_xlabel('Time')
# ax4.set_xlim(start_time + dt*dt_num_offset, start_time + time - dt)
# ax4.set_ylim(-10, 10)


# plt.savefig('results.png')
plt.show()
