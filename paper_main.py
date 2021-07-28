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

from models.control.bldg_grid_agg_data_driven_mdl_large import bldg_grid_agg_data_driven_mdl_large
from models.control.bldg_grid_agg_data_driven_mdl_med import bldg_grid_agg_data_driven_mdl_med
from models.control.bldg_grid_agg_data_driven_mdl_small import bldg_grid_agg_data_driven_mdl_small
from models.simulation.bldg_sim_mdl_large import bldg_sim_mdl_large
from models.simulation.bldg_sim_mdl_med import bldg_sim_mdl_med
from models.simulation.bldg_sim_mdl_small import bldg_sim_mdl_small
from models.control.grid_aggregator import grid_aggregator

start_time = 9*60 - 15   # Start time in minutes; 700
dt = 1              # Time-step in minutes

tmp = opt.LCDMPC(start_time, dt)

time = 8*60 + 15     # Length of simulation in minutes
horiz_len = 5   # Prediction horizion length
commuincation_iterations = 12 # number of communications between subsystems
Beta = 0.1      # Convex combination parameter for control action

time_array = np.arange(start_time, (start_time + time), dt)

# bldg1_disturb_file = 'input/ROM_simulation_data_interp.csv'
bldg1_small_disturb_file = 'input/ROM_simulation_data_small_office.csv'
bldg1_disturb_file = 'input/ROM_simulation_data_large_office_denver.csv'

num_buildings_large = 1
num_buildings_medium = 2
num_buildings_small = 0
num_buildings_total = num_buildings_large + num_buildings_medium + num_buildings_small

ms_dot_large = 8.0
T_sa_large = 12.8
T_oa_large = 28.
T_z_large = 22.5
T_e_large = 20.

ms_dot_medium = 1.0
T_sa_medium = 12.8
T_oa_medium = 28.95
T_z_medium = 20.5
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

refs_large = [[-1.5], [20.], [0.0]]
refs_medium = [[1.0], [20.], [0.0]]
refs_small = [[0], [20.], [0.0]]

disturbance_data = pd.read_csv(bldg1_disturb_file)
Toa_horiz = disturbance_data.iloc[start_time: start_time + int(time/dt) + horiz_len]['T_outside'].values
Toa_horiz_normed = Toa_horiz/Toa_horiz[0]

np.random.seed(1)
grid_agg_ref = np.random.normal(6*num_buildings_small + 20*num_buildings_medium + 40*num_buildings_large, 5.0, int(time/dt) + horiz_len)#*Toa_horiz_normed

refs_grid_total = pd.DataFrame()
for i in range(int(time/dt) + horiz_len):
    refs_grid_total = refs_grid_total.append(
        {
            'time': start_time + i,
            'grid_ref': [[grid_agg_ref[i]]] + [[0.] for i in range(num_buildings_total)]
        },
        ignore_index=True
    )

bldg_optoptions_large = {
    # 'Major feasibility tolerance': 1e1,
    'Print file': 'SNOPT_bldg_lg_print.out',
    'Summary file': 'SNOPT_bldg_lg_summary.out',
    'Proximal iterations limit': 1000,
}
bldg_optoptions_med = {
    # 'Major feasibility tolerance': 1e1,
    'Print file': 'SNOPT_bldg_med_print.out',
    'Summary file': 'SNOPT_bldg_med_summary.out',
    'Proximal iterations limit': 1000,
}
grid_optoptions = {
    # 'Major feasibility tolerance': 1e3,
    'Print file': 'SNOPT_grid_print.out',
    'Summary file': 'SNOPT_grid_summary.out',
    'Proximal iterations limit': 1000,
}

num_downstream1 = num_buildings_total
num_upstream1 = 0

building_control_models = []
building_truth_models = []

Qint_scale = np.random.normal(1.3, 0.2, num_buildings_total).tolist()
Qsol_scale = np.random.normal(1.5, 0.5, num_buildings_total).tolist()
# Qsol_scale = np.ones(np.shape(Qsol_scale)).tolist()

Qint_offset = [0.0]*num_buildings_total
Qsol_offset = [0.0]*num_buildings_total

Qint_std = [1.0]*num_buildings_total
Qsol_std = [1.0]*num_buildings_total

energy_red_weight = [0.0]*num_buildings_total

for i in range(num_buildings_large):
    building_control_models.append(
        bldg_grid_agg_data_driven_mdl_large(
            ms_dot_large, T_sa_large, T_z_large, horiz_len, energy_red_weight[i],
            Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
            Qint_offset[i], Qsol_offset[i]
        )
    )
    building_truth_models.append(
        bldg_sim_mdl_large(dt/60, ms_dot_large, T_sa_large, T_z_large, T_e_large, start_time,
        Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
        Qint_offset[i], Qsol_offset[i])
    )

for i in range(num_buildings_medium):
    building_control_models.append(
        bldg_grid_agg_data_driven_mdl_med(
            ms_dot_medium, T_sa_medium, T_z_medium, horiz_len, energy_red_weight[i],
            Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
            Qint_offset[i], Qsol_offset[i]
        )
    )
    building_truth_models.append(
        bldg_sim_mdl_med(dt/60, ms_dot_medium, T_sa_medium, T_z_medium, T_e_medium, start_time,
        Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
        Qint_offset[i], Qsol_offset[i])
    )

for i in range(num_buildings_small):
    building_control_models.append(
        bldg_grid_agg_data_driven_mdl_small(
            ms_dot_small, T_sa_small, T_z_small, horiz_len, energy_red_weight[i],
            Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
            Qint_offset[i], Qsol_offset[i]
        )
    )
    building_truth_models.append(
        bldg_sim_mdl_small(dt/12, ms_dot_small, T_sa_small, T_z_small, T_e_small, start_time,
        Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
        Qint_offset[i], Qsol_offset[i])
    )

grid_agg1_cont = grid_aggregator(horiz_len, num_downstream1)
grid_agg1_truth = grid_aggregator(horiz_len, num_downstream1)

tmp.build_subsystem(0, grid_agg1_cont, grid_agg1_truth, 
    inputs, outputs1, horiz_len, Beta, bldg1_disturb_file, refs_total=refs_grid_total,
    optOptions=grid_optoptions)

for i in range(num_buildings_large):
    print('i large: ', i + 1)
    tmp.build_subsystem(i + 1, building_control_models[i], building_truth_models[i],
    inputs_large, outputs1, horiz_len, Beta, bldg1_disturb_file, refs=refs_large,
    optOptions=bldg_optoptions_large)

for i in range(num_buildings_medium):
    print('i medium: ', i + 1 + num_buildings_large)
    ind = i + 1 + num_buildings_large
    tmp.build_subsystem(ind, building_control_models[ind-1], building_truth_models[ind-1],
    inputs_medium, outputs1, horiz_len, Beta, bldg1_disturb_file, refs=refs_medium,
    optOptions=bldg_optoptions_med)

for i in range(num_buildings_small):
    print('i small: ', i + 1 + num_buildings_large + num_buildings_medium)
    ind = i + 1 + num_buildings_large + num_buildings_medium
    tmp.build_subsystem(ind, building_control_models[ind-1], building_truth_models[ind-1],
    inputs_small, outputs1, horiz_len, Beta, bldg1_small_disturb_file, refs=refs_small,
    optOptions=bldg_optoptions)

connections = [[0, i+1] for i in range(num_buildings_total)] + \
              [[i+1, 0] for i in range(num_buildings_total)]

# connections = None
tmp.build_interconnections(interconnections=connections)

outputs_all = []
controls_all = []
gamma_all = []

for i in range(int(time/dt)):

    print('+++++++++++++++++++++++++++++')
    print('time iteration: ', i)
    print('+++++++++++++++++++++++++++++')

    # TODO: Need to map states to updated state (LPV like)

    tmp.relinearize_subsystem_models()

    gamma_comm = []
    for j in range(commuincation_iterations):
        # Communication step
        tmp.communicate()
        # Optimize all subsystems (individual objective functions)
        tmp.optimize_all()
        # Convex summation of control parameters (for stability)
        tmp.convex_sum_cont()
        # Update Z's for downstream subsystems
        tmp.update_downstream_outputs()

        gamma_comm.append(tmp.calculate_sensitivities())

        # print('==============================')
        print('communication iteration: ', j)
        # print('==============================')
    # Update state equations for subsystems
    tmp.update_states()
    # Update outputs of subsystems
    tmp.update_subsystem_outputs()

    # Gather outputs for plotting purposes
    outputs = tmp.simulate_truth_model()
    outputs_all.append(outputs)

    # Update control model's filter
    tmp.update_control_filter()
    # Update values for linearization
    tmp.update_inputs_for_linearization()
    # Get updated forecast inputs
    tmp.update_forecast_inputs()
    # Gather control actions for plotting purposes
    controls_all.append([[subsys.uConv] for subsys in tmp.subsystems])
    gamma_all.append(gamma_comm)

import pickle

pickle.dump([tmp, outputs_all, controls_all, gamma_all], open('results.p', 'wb'))

dt_num_offset = 15

gamma_gridagg_all_plot = []
for i in range(num_buildings_total):
    gamma_gridagg_all_plot.append(np.array([val[-1][0][i] for val in gamma_all]).flatten())

gamma_gridagg_plot = []
for i in range(num_buildings_total):
    gamma_gridagg_plot.append(np.array([val[0][i] for val in gamma_comm]).flatten())

fig, axes = plt.subplots(2, 2, figsize=(16,10))
ax = axes[0, 0]
for i in range(len(gamma_gridagg_all_plot)):
    ax.plot(time_array[dt_num_offset:], gamma_gridagg_all_plot[i][dt_num_offset:], label=str(i))
ax.set_title('Aggregator Gamma Sens.')
xticks = ax.get_xticks()
ax.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax.set_ylabel('gamma')
ax.set_xlabel('Time')
ax.legend()

ax = axes[0, 1]
for i in range(len(gamma_gridagg_plot)):
    ax.plot(gamma_gridagg_plot[i], label=str(i))
ax.set_title('Aggregator Gamma Sens. - Last Iteration')
ax.set_ylabel('gamma')
ax.set_xlabel('Communication Iteration')
ax.set_xticks(np.arange(0, commuincation_iterations, 1))
ax.legend()


gamma_bldg_all_plot = []
for i in range(num_buildings_total):
    gamma_bldg_all_plot.append(np.array([val[-1][i+1][0] for val in gamma_all]).flatten())

gamma_bldg_plot = []
for i in range(num_buildings_total):
    gamma_bldg_plot.append(np.array([val[i+1][0] for val in gamma_comm]).flatten())
# print(gamma_bldg_plot)

ax = axes[1, 0]
for i in range(len(gamma_bldg_all_plot)):
    ax.plot(time_array[dt_num_offset:], gamma_bldg_all_plot[i][dt_num_offset:], label=str(i))
ax.set_title('Building Gamma Sens.')
xticks = ax.get_xticks()
ax.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax.set_ylabel('gamma')
ax.set_xlabel('Time')
ax.legend()

ax = axes[1, 1]
for i in range(len(gamma_bldg_plot)):
    plt.plot(gamma_bldg_plot[i], label=str(i))
ax.set_title('Building Gamma Sens. - Last Iteration')
ax.set_ylabel('gamma')
ax.set_xlabel('Communication Iteration')
ax.set_xticks(np.arange(0, commuincation_iterations, 1))
ax.legend()

plt.savefig('gammas.png', bbox_inches='tight')


np.set_printoptions(suppress=True)

# gamma_plot = []
# for i in range(commuincation_iterations):
#     gamma_plot.append()

plot_temps = []
for i in range(num_buildings_total):
    plot_temps.append([val[i+1:][0][0][0][0] for val in outputs_all])

plot_bldg_powers = []
for i in range(num_buildings_total):
    plot_bldg_powers.append([val[i+1:][0][1][0][0] for val in outputs_all])

# print(outputs_all)
# print('-------------')
# print(outputs_all[0][0])
# print(outputs_all[0][0][0])

grid_prefs_ind = []
for i in range(num_buildings_total):
    grid_prefs_ind.append([val[0][i][0] for val in outputs_all])

# print(grid_prefs_ind)
# lkj

total_power = []
grid_prefs_total = []

for i in range(len(outputs_all)):
    sum_of_powers = np.sum([val[1] for val in outputs_all[i][1:]])
    sum_of_grid_prefs = np.sum(outputs_all[i][0][:])
    total_power.append(sum_of_powers)
    grid_prefs_total.append(sum_of_grid_prefs)

T_lower = 20.0
T_upper = 23.3
P_lower = 0.0
P_upper = 100.0

fig, axes = plt.subplots(2, 2, figsize=(16,10))

disturbance_data = pd.read_csv(bldg1_disturb_file)

ax2 = axes[0, 0]
bldg_temp_legend = []
for i in range(num_buildings_total):
    ax2.plot(
        time_array[dt_num_offset:], np.array(plot_temps[i]).flatten()[dt_num_offset:]
    )
    bldg_temp_legend.append('Bldg ' + str(i))
ax2.plot(
    time_array[dt_num_offset:],
    disturbance_data.iloc[
        start_time + dt_num_offset: start_time + int(time/dt)
    ]['T_outside'].values, color='gray'
)
ax2.plot(
    time_array[dt_num_offset:], T_lower*np.ones(len(time_array))[dt_num_offset:], '--k'
)
ax2.plot(
    time_array[dt_num_offset:], T_upper*np.ones(len(time_array))[dt_num_offset:], '--k'
)
ax2.legend(bldg_temp_legend + ['Outdoor Air Temp'])
xticks = ax2.get_xticks()
ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax2.set_title('Building Temperatures')
ax2.set_ylabel('Temperature [C]')
ax2.set_xlabel('Time')

colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
ax2 = axes[1, 0]
bldg_power_legend = []
for i in range(num_buildings_total):
    ax2.plot(
        time_array[dt_num_offset:],
        np.array(plot_bldg_powers[i]).flatten()[dt_num_offset:],
        color=colors[i]
    )
    bldg_power_legend.append('Bldg ' + str(i))
for i in range(num_buildings_total):
    ax2.plot(
        time_array[dt_num_offset:],
        np.array(grid_prefs_ind[i]).flatten()[dt_num_offset:],
        '--',
        color=colors[i]
    )
    bldg_power_legend.append('Pref ' + str(i))
# ax2.plot(time_array[dt_num_offset:], pref1[dt_num_offset:], '--', color='tab:blue')
# bldg_power_legend.append('pref1')
# ax2.plot(time_array[dt_num_offset:], pref2[dt_num_offset:], '--', color='tab:orange')
# bldg_power_legend.append('pref2')
ax2.legend(bldg_power_legend)
xticks = ax2.get_xticks()
ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax2.set_title('Building Powers')
ax2.set_ylabel('Power [kW]')
ax2.set_xlabel('Time')

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
ax4.plot(time_array[dt_num_offset:], grid_agg_ref[dt_num_offset-1:-horiz_len-1], '-.k')
ax4.legend(['Bldg Power', 'Power Ref Stpts', 'Grid Power Ref'])
xticks = ax4.get_xticks()
ax4.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax4.set_title('Total Power')
ax4.set_ylabel('Power [kw]')
ax4.set_xlabel('Time')

ax4 = axes[1, 1]
print(np.shape(grid_agg_ref))
print(np.shape(total_power))
print(grid_agg_ref)
print(total_power)
ax4.plot(
    time_array[dt_num_offset:],
    np.array(
        (
            np.array(grid_agg_ref[dt_num_offset-1:-horiz_len-1]) - np.array(total_power).flatten()[dt_num_offset:]
        ) / np.array(grid_agg_ref[dt_num_offset-1:-horiz_len-1])*100
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


