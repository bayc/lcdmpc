# Copyright 2020 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not 
# use this file except in compliance with the License. You may obtain a copy of 
# the License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT 
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the 
# license for the specific language governing permissions and limitations under 
# the License.

import os
import lcdmpc as opt
import numpy as np
# import matplotlib
# matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
import pandas as pd


bldg_ctrl_imports = []
for file in os.listdir('models/control'):
    if file.startswith('bldg_ctrl_pena_'):
        bldg_ctrl_imports.append(file[:-3])

bldg_sim_imports = []
for file in os.listdir('models/simulation'):
    if file.startswith('bldg_sim_pena_'):
        bldg_sim_imports.append(file[:-3])

for i in range(len(bldg_ctrl_imports)):
    exec('from models.control.' + bldg_ctrl_imports[i] + ' import bldg_ctrl_pena as ' + bldg_ctrl_imports[i])
    exec('from models.simulation.' + bldg_sim_imports[i] + ' import bldg_sim_pena as ' + bldg_sim_imports[i])


# from models.control.bldg_grid_agg_data_driven_bldg60 import bldg_grid_agg_data_driven_bldg60
# from models.control.bldg_grid_agg_data_driven_bldg62 import bldg_grid_agg_data_driven_bldg62
# from models.control.bldg_pena_36 import bldg_pena_36
# from models.control.bldg_pena_51 import bldg_pena_51
# from models.control.bldg_pena_52 import bldg_pena_52
# from models.control.bldg_pena_73 import bldg_pena_73
from models.control.bldg_grid_agg_data_driven_mdl_large import bldg_grid_agg_data_driven_mdl_large
from models.control.bldg_grid_agg_data_driven_mdl_med import bldg_grid_agg_data_driven_mdl_med
from models.control.bldg_grid_agg_data_driven_mdl_small import bldg_grid_agg_data_driven_mdl_small
# from models.simulation.bldg_sim_pena_36 import bldg_sim_pena_36
# from models.simulation.bldg_sim_pena_51 import bldg_sim_pena_51
# from models.simulation.bldg_sim_pena_52 import bldg_sim_pena_52
# from models.simulation.bldg_sim_pena_73 import bldg_sim_pena_73
# from models.simulation.bldg_sim_mdl_bldg62 import bldg_sim_mdl_bldg62
from models.simulation.bldg_sim_mdl_large import bldg_sim_mdl_large
from models.simulation.bldg_sim_mdl_med import bldg_sim_mdl_med
from models.simulation.bldg_sim_mdl_small import bldg_sim_mdl_small
from models.control.grid_aggregator import grid_aggregator
from models.control.sub_grid_aggregator import sub_grid_aggregator

dt_num_offset = 1

start_time = 32*60 - dt_num_offset   # Start time in minutes; 700
dt = 1              # Time-step in minutes

lcdmpc = opt.LCDMPC(start_time, dt)

time = 30 + dt_num_offset     # Length of simulation in minutes
horiz_len = 5   # Prediction horizion length
commuincation_iterations = 12 # number of communications between subsystems
Beta = 0.4      # Convex combination parameter for control action

time_array = np.arange(start_time, (start_time + time), dt)

# bldg1_disturb_file = 'input/ROM_simulation_data_interp.csv'
bldg1_small_disturb_file = 'input/ROM_simulation_data_small_office.csv'
bldg1_disturb_file = 'input/ROM_simulation_data_large_office_denver.csv'
# bldg1_disturb_file = 'models/model_identification/building_60_1min.csv'

# List of buildings to simulate
# building_ids = [1, 5, 6, 7]
building_ids = [
    [1, 5, 53, 60, 61],
    [52, 67, 64, 128],#, 21],
    [20, 17, 14, 15, 16],
]

from collections.abc import Iterable

def flatten(xs):
    for x in xs:
        if isinstance(x, Iterable) and not isinstance(x, (str, bytes)):
            yield from flatten(x)
        else:
            yield x

tmp = flatten(building_ids)
building_ids_flat = [val for val in tmp]

# Grab the mean building temps for the control models
cases_directory = 'models/model_identification/'

control_model_base_name = 'bldg_ctrl_pena_'
sim_model_base_name = 'bldg_sim_pena_'

bldg_mean_temps = []
bldg_ms_dot = []
bldg_T_sa = []
bldg_T_oa = []
bldg_T_z = []
bldg_T_e = []
inputs_all = []

T_oa_init = 25.0
T_z_init = 21.5

for i, id in enumerate(building_ids_flat):
    df_model = pd.read_excel(cases_directory + 'building_' + str(id) + '.xlsx', engine='openpyxl')

    bldg_mean_temps.append(df_model['Means'][1])
    bldg_ms_dot.append(df_model['mdot_limits'][1] * 0.1)
    bldg_T_sa.append(df_model['T discharge limits'][0])
    bldg_T_oa.append(T_oa_init)
    bldg_T_z.append(T_z_init)
    bldg_T_e.append(np.mean([df_model['Means'][0], df_model['Means'][1]]))
    inputs_all.append(
        [
            bldg_ms_dot[i],
            bldg_T_sa[i],
            bldg_T_oa[i],
            bldg_T_z[i],
        ]
    )

num_buildings_total = len(building_ids_flat)
num_sub_grid_aggs = len(building_ids)
num_of_grid_aggs_total = num_sub_grid_aggs + 1

Q_int = 8.0
Q_solar = 15.0

outputs1 = [1]

temp_setpoint = 22.22

refs_bldg_all = []
for i in range(len(building_ids_flat)):
    refs_bldg_all.append([[temp_setpoint - bldg_mean_temps[i]], [20.0], [0.0], [0.0]])

disturbance_data = pd.read_csv(bldg1_disturb_file)
Toa_horiz = disturbance_data.iloc[start_time: start_time + int(time/dt) + horiz_len]['T_outdoor'].values
Toa_horiz_normed = Toa_horiz/Toa_horiz[0]

np.random.seed(1)
# grid_agg_ref = np.random.normal(6*num_buildings_small + 20*num_buildings_medium + 10*num_buildings_large, 0.0, int(time/dt) + horiz_len)*Toa_horiz_normed
grid_agg_ref = np.random.normal(11*len(building_ids_flat), 2.0, int(10*60/dt) + horiz_len)#*Toa_horiz_normed

refs_grid_total = pd.DataFrame()
# for i in range(int(time/dt) + horiz_len):
#     refs_grid_total = pd.concat(
#         [
#             refs_grid_total,
#             pd.DataFrame({
#                 'time': start_time + i,
#                 # 'grid_ref': [[grid_agg_ref[i]]] + [[0.] for i in range(int(num_buildings_total))],
#                 'grid_ref': [[grid_agg_ref[i]]] + [[0.] for i in range(int(num_sub_grid_aggs))],
#             })
#         ],
#         ignore_index=True,
#     )

for i in range(int(time/dt) + horiz_len):
    refs_grid_total = refs_grid_total.append(
        {
            'time': start_time + i,
            # 'grid_ref': [[grid_agg_ref[i]]] + [[0.] for i in range(int(num_buildings_total))],
            'grid_ref': [[grid_agg_ref[i]]] + [[0.] for i in range(int(num_sub_grid_aggs))],
        },
        ignore_index=True
    )

bldg_optoptions = {
    # 'Major feasibility tolerance': 1e1,
    'Print file': 'SNOPT_bldg_lg_print.out',
    'Summary file': 'SNOPT_bldg_lg_summary.out',
    'Proximal iterations limit': 1000,
}
grid_optoptions = {
    # 'Major feasibility tolerance': 1e3,
    'Print file': 'SNOPT_grid_print.out',
    'Summary file': 'SNOPT_grid_summary.out',
    'Proximal iterations limit': 1000,
}

building_control_models = []
building_truth_models = []

np.random.seed(1)
Qint_scale = np.random.normal(1.3, 0.2, num_buildings_total).tolist()
np.random.seed(1)
Qsol_scale = np.random.normal(1.5, 0.5, num_buildings_total).tolist()

Qint_scale = np.ones(np.shape(Qint_scale)).tolist()
Qsol_scale = np.ones(np.shape(Qsol_scale)).tolist()

Qint_offset = [0.0]*num_buildings_total
Qsol_offset = [0.0]*num_buildings_total

Qint_std = [1.0]*num_buildings_total
Qsol_std = [1.0]*num_buildings_total

energy_red_weight = [0.0]*num_buildings_total

# Initialize building control and truth models
i = 0
for j in range(len(building_ids_flat)):
    str1 = 'building_control_models.append(bldg_ctrl_pena_' + str(building_ids_flat[j]) + '(bldg_ms_dot[j], bldg_T_sa[j], bldg_T_z[j], horiz_len, energy_red_weight[i], Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i], Qint_offset[i], Qsol_offset[i]))'
    eval(str1)

    str2 = 'building_truth_models.append(bldg_sim_pena_' + str(building_ids_flat[j]) + '(dt/60, bldg_ms_dot[j], bldg_T_sa[j], bldg_T_z[j], bldg_T_e[j], start_time, Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],  Qint_offset[i], Qsol_offset[i]))'
    eval(str2)

# Initialize sub grid aggregator control and truth models
for i, cluster in enumerate(building_ids):
    str1 = 'sub_grid_agg_cont_' + str(i) + ' = sub_grid_aggregator(horiz_len, len(cluster) + 1, len(cluster) + 1)'
    str2 = 'sub_grid_agg_truth_' + str(i) + ' = sub_grid_aggregator(horiz_len, len(cluster) + 1, len(cluster) + 1)'
    
    exec(str1)
    exec(str2)

# Initialize grid aggregator control and truth models
grid_agg_cont_1 = grid_aggregator(horiz_len, num_sub_grid_aggs)
grid_agg_truth_1 = grid_aggregator(horiz_len, num_sub_grid_aggs)

# Build the subsystems
lcdmpc.build_subsystem(0, grid_agg_cont_1, grid_agg_truth_1, 
    inputs_all[0], outputs1, horiz_len, Beta, bldg1_disturb_file, refs_total=refs_grid_total,
    optOptions=grid_optoptions)

for i in range(num_sub_grid_aggs):
    sub_grid_agg_ref = [[0.0] for _ in range(len(building_ids[i]))]
    str1 = 'lcdmpc.build_subsystem(i + 1, sub_grid_agg_cont_' + str(i) + ', sub_grid_agg_truth_' + str(i) + ', inputs_all[0], outputs1, horiz_len, Beta, bldg1_disturb_file, refs=sub_grid_agg_ref, optOptions=grid_optoptions)'

    eval(str1)

for i in range(len(building_ids_flat)):
    lcdmpc.build_subsystem(i + num_sub_grid_aggs + 1, building_control_models[i], building_truth_models[i],
    inputs_all[i], outputs1, horiz_len, Beta, bldg1_disturb_file, refs=refs_bldg_all[i],
    optOptions=bldg_optoptions)

connections = []

for i in np.arange(1, len(building_ids) + 1):
    connections.append([0, i])
    connections.append([i, 0])

ii = 0
for i, cluster in enumerate(building_ids):
    for j in range(len(cluster)):
        connections.append([i + 1 , 1 + num_sub_grid_aggs + ii])
        connections.append([1 + num_sub_grid_aggs + ii, i + 1])
        ii += 1

lcdmpc.build_interconnections(interconnections=connections)

outputs_all = []
disturbance_all = []
controls_all = []
gamma_all = []

# lcdmpc.calc_stability()
# lkj

for i in range(int(time/dt)):

    print('+++++++++++++++++++++++++++++')
    print('time iteration: ', i)
    print('+++++++++++++++++++++++++++++')

    # TODO: Need to map states to updated state (LPV like)

    lcdmpc.relinearize_subsystem_models()

    gamma_comm = []
    for j in range(commuincation_iterations):
        # Communication step
        lcdmpc.communicate()
        # Optimize all subsystems (individual objective functions)
        lcdmpc.optimize_all()
        # Convex summation of control parameters (for stability)
        lcdmpc.convex_sum_cont()
        # Update Z's for downstream subsystems
        lcdmpc.update_downstream_outputs()

        gamma_comm.append(lcdmpc.calculate_sensitivities())

        # print('==============================')
        print('communication iteration: ', j)
        # print('==============================')
    # Update state equations for subsystems
    lcdmpc.update_states()
    # Update outputs of subsystems
    lcdmpc.update_subsystem_outputs()

    # Gather outputs for plotting purposes
    outputs = lcdmpc.simulate_truth_model()
    outputs_all.append(outputs)

    # Update control model's filter
    lcdmpc.update_control_filter()
    # Update values for linearization
    lcdmpc.update_inputs_for_linearization()
    # Get updated forecast inputs
    lcdmpc.update_forecast_inputs()
    # Gather control actions for plotting purposes
    disturbance_all.append([[subsys.d] for subsys in lcdmpc.subsystems])
    controls_all.append([[subsys.uConv] for subsys in lcdmpc.subsystems])
    gamma_all.append(gamma_comm)

import pickle

pickle.dump([lcdmpc, outputs_all, disturbance_all, controls_all, gamma_all], open('results.p', 'wb'))

# time_array = time_array - 24*60

# lcdmpc, outputs_all, disturbance_all, controls_all, gamma_all = pickle.load(
#     open('results.p', 'rb')
# )

gamma_gridagg_all_plot = []
for i in range(num_buildings_total):
    gamma_gridagg_all_plot.append(np.array([val[-1][0][i] for val in gamma_all]).flatten())

gamma_gridagg_plot = []
for i in range(num_buildings_total):
    gamma_gridagg_plot.append(np.array([val[0][i] for val in gamma_all[-1]]).flatten())

fig, axes = plt.subplots(2, 2, figsize=(16,10))
ax = axes[0, 0]
for i in range(len(gamma_gridagg_all_plot)):
    ax.plot(time_array[dt_num_offset:], gamma_gridagg_all_plot[i][dt_num_offset:], label=str(i))
ax.set_title('Aggregator Gamma Sensitivity')
xticks = ax.get_xticks()
ax.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax.set_ylabel('gamma')
ax.set_xlabel('Time')
ax.legend(['Large Office Building 0', 'Medium Office Building 1', 'Medium Office Building 2'])

ax = axes[0, 1]
for i in range(len(gamma_gridagg_plot)):
    ax.plot(gamma_gridagg_plot[i], label=str(i))
ax.set_title('Aggregator Gamma Sensitivity - Last Iteration')
ax.set_ylabel('gamma')
ax.set_xlabel('Communication Iteration')
ax.set_xticks(np.arange(0, commuincation_iterations, 1))
ax.legend(['Large Office Building 0', 'Medium Office Building 1', 'Medium Office Building 2'])


gamma_bldg_all_plot = []
for i in range(num_buildings_total):
    gamma_bldg_all_plot.append(np.array([val[-1][i+1][0] for val in gamma_all]).flatten())

gamma_bldg_plot = []
for i in range(num_buildings_total):
    gamma_bldg_plot.append(np.array([val[i+1][0] for val in gamma_all[-1]]).flatten())
# print(gamma_bldg_plot)

ax = axes[1, 0]
for i in range(len(gamma_bldg_all_plot)):
    ax.plot(time_array[dt_num_offset:], gamma_bldg_all_plot[i][dt_num_offset:], label=str(i))
ax.set_title('Building Gamma Sensitivity')
xticks = ax.get_xticks()
ax.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax.set_ylabel('gamma')
ax.set_xlabel('Time')
ax.legend(['Large Office Building 0', 'Medium Office Building 1', 'Medium Office Building 2'])

ax = axes[1, 1]
for i in range(len(gamma_bldg_plot)):
    plt.plot(gamma_bldg_plot[i], label=str(i))
ax.set_title('Building Gamma Sensitivity - Last Iteration')
ax.set_ylabel('gamma')
ax.set_xlabel('Communication Iteration')
ax.set_xticks(np.arange(0, commuincation_iterations, 1))
ax.legend(['Large Office Building 0', 'Medium Office Building 1', 'Medium Office Building 2'])

plt.savefig('gammas.png', bbox_inches='tight')


np.set_printoptions(suppress=True)

# gamma_plot = []
# for i in range(commuincation_iterations):
#     gamma_plot.append()

plot_temps = []
for i in range(num_buildings_total):
    plot_temps.append([val[i+num_of_grid_aggs_total:][0][0][0][0] for val in outputs_all])

plot_bldg_powers = []
for i in range(num_buildings_total):
    plot_bldg_powers.append([val[i+num_of_grid_aggs_total:][0][1][0][0] for val in outputs_all])

print(outputs_all[-1])
# print(outputs_all[0][3])
# print('-------------')
# print(outputs_all[0][0])
# print(outputs_all[0][0][0])

grid_agg_prefs_ind = []
for i in range(int(num_sub_grid_aggs)):
    grid_agg_prefs_ind.append([val[0][i][0] for val in outputs_all])

sub_grid_agg_prefs_ind = []
num_buildings_for_each_sub_agg = [4, 4, 5, 7, 5]
for j in range(int(num_sub_grid_aggs)):
    for i in range(int(num_buildings_for_each_sub_agg[j])):
        sub_grid_agg_prefs_ind.append([val[j+1][i][0] for val in outputs_all])

# print(outputs_all)
# lkj

total_power = []
grid_prefs_total = []

for i in range(len(outputs_all)):
    sum_of_powers = np.sum([val[1] for val in outputs_all[i][num_of_grid_aggs_total:]])
    sum_of_grid_prefs = np.sum(outputs_all[i][0][:])
    total_power.append(sum_of_powers)
    grid_prefs_total.append(sum_of_grid_prefs)

T_lower = 70.0 # 21.11
T_upper = 74.0 # 23.33
P_lower = 0.0
P_upper = 100.0

fig, axes = plt.subplots(2, 2, figsize=(16,10))

disturbance_data = pd.read_csv(bldg1_disturb_file)

colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan', 'tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan', 'tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan', 'tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
# colors = ['tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:red']
ax2 = axes[0, 0]
bldg_temp_legend = ['Outdoor Air Temp']
ax2.plot(
    time_array[dt_num_offset:],
    disturbance_data.iloc[
        start_time + dt_num_offset: start_time + int(time/dt)
    ]['T_outdoor'].values * 9 / 5 +32, color='gray'
)
for i in range(num_buildings_total):
    ax2.plot(
        time_array[dt_num_offset:], np.array(plot_temps[i]).flatten()[dt_num_offset:] * 9 / 5 +32,
        color=colors[i]
    )
    # bldg_temp_legend.append('Large Office Building ' + str(i))

ax2.plot(
    time_array[dt_num_offset:], T_lower*np.ones(len(time_array))[dt_num_offset:], '--k'
)
ax2.plot(
    time_array[dt_num_offset:], T_upper*np.ones(len(time_array))[dt_num_offset:], '--k'
)
ax2.plot(time_array[dt_num_offset:], (temp_setpoint)*np.ones(len(time_array))[dt_num_offset:] * 9 / 5 +32, '--r')
# ax2.legend(bldg_temp_legend + ['Outdoor Air Temp'])
ax2.legend(bldg_temp_legend)
xticks = ax2.get_xticks()
ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax2.set_title('Building Temperatures')
ax2.set_ylabel('Temperature [F]')
ax2.set_xlabel('Time')

# ax2 = axes[1, 0]
# bldg_temp_legend = ['Emissions Data']

# ax2.plot(
#     time_array[dt_num_offset:],
#     disturbance_data.iloc[
#         start_time + dt_num_offset: start_time + int(time/dt)
#     ]['emmissions'].values, color='gray'
# )

# xticks = ax2.get_xticks()
# ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
# ax2.set_title('Emissions Rate of Electricity Used')
# ax2.set_ylabel('CO2 Emissions [kg/MWh]')
# ax2.set_xlabel('Time')

# colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan', 'tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
# colors = ['tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:red']
ax2 = axes[1, 0]
bldg_power_legend = []
for i in range(num_buildings_total):
    ax2.plot(
        time_array[dt_num_offset:],
        np.array(plot_bldg_powers[i]).flatten()[dt_num_offset:],
        color=colors[i]
    )
    bldg_power_legend.append('P Building ' + str(i))
for i in range(num_buildings_total):
    ax2.plot(
        time_array[dt_num_offset:],
        np.array(sub_grid_agg_prefs_ind[i]).flatten()[dt_num_offset:],
        '--',
        color=colors[i]
    )
    bldg_power_legend.append(r'$P_{ref}$ Building ' + str(i))
# ax2.plot(time_array[dt_num_offset:], pref1[dt_num_offset:], '--', color='tab:blue')
# bldg_power_legend.append('pref1')
# ax2.plot(time_array[dt_num_offset:], pref2[dt_num_offset:], '--', color='tab:orange')
# bldg_power_legend.append('pref2')
# ax2.legend(bldg_power_legend)
xticks = ax2.get_xticks()
ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax2.set_title('Building Powers')
ax2.set_ylabel('Power [kW]')
ax2.set_xlabel('Time')

# emissions_used = np.multiply(
#     disturbance_data.iloc[start_time + dt_num_offset: start_time + int(time/dt)]['emmissions'].values,
#     np.array(total_power).flatten()[dt_num_offset:]/1e3,
# )

# from numpy import trapz

# total_emissions = trapz(emissions_used/60, dx=1)
# total_power = trapz(np.array(total_power).flatten()[dt_num_offset:], dx=1)
# print('total emissions: ', total_emissions)
# print('total power: ', total_power)

# ax4 = axes[0, 1]
# ax4.plot(
#     time_array[dt_num_offset:],
#     emissions_used,
#     '-b'
# )
# # ax4.plot(
# #     time_array[dt_num_offset:],
# #     np.array(grid_prefs_total).flatten()[dt_num_offset:],
# #     '-r'
# # )
# # ax4.plot(time_array[dt_num_offset:], grid_agg_ref[dt_num_offset-1:time-1], '-.k')
# ax4.legend(['CO2 Emissions'])
# xticks = ax4.get_xticks()
# ax4.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
# ax4.set_title('Instantaneous CO2 Emissions')
# ax4.set_ylabel('CO2 Emissions')
# ax4.set_xlabel('Time')

ax4 = axes[0, 1]
ax4.plot(
    time_array[dt_num_offset:],
    np.array(total_power).flatten()[dt_num_offset:],
    '-b'
)
ax4.plot(
    time_array[dt_num_offset:],
    np.array(grid_prefs_total).flatten()[dt_num_offset:],
    '-r'
)
ax4.plot(time_array[dt_num_offset:], grid_agg_ref[dt_num_offset-1:time-1], '-.k')
ax4.legend(['Total Building Power', 'Total Power Ref. Setpoints', 'Grid Power Ref.'])
xticks = ax4.get_xticks()
ax4.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax4.set_title('Total Power')
ax4.set_ylabel('Power [kw]')
ax4.set_xlabel('Time')

ax4 = axes[1, 1]
# print(np.shape(grid_agg_ref))
# print(np.shape(total_power))
# print(grid_agg_ref)
# print(total_power)
ax4.plot(
    time_array[dt_num_offset:],
    np.array(
        (
            np.array(grid_agg_ref[dt_num_offset-1:time-1]) - np.array(total_power).flatten()[dt_num_offset:]
        ) / np.array(grid_agg_ref[dt_num_offset-1:time-1])*100
    ),
    '-r'
)
xticks = ax4.get_xticks()
ax4.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax4.set_title('Power Error')
ax4.set_ylabel('Error [%]')
ax4.set_xlabel('Time')

plt.savefig('results.png', bbox_inches='tight')

plt.show()


