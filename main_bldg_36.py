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
# import matplotlib
# matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
import pandas as pd

# from models.control.bldg_grid_agg_data_driven_bldg60 import bldg_grid_agg_data_driven_bldg60
# from models.control.bldg_grid_agg_data_driven_bldg62 import bldg_grid_agg_data_driven_bldg62
from models.control.bldg_ctrl_pena_1 import bldg_ctrl_pena as bldg_ctrl_pena_1
from models.control.bldg_ctrl_pena_9 import bldg_ctrl_pena as bldg_ctrl_pena_9
from models.control.bldg_ctrl_pena_10 import bldg_ctrl_pena as bldg_ctrl_pena_10
from models.control.bldg_ctrl_pena_14 import bldg_ctrl_pena as bldg_ctrl_pena_14
from models.control.bldg_ctrl_pena_15 import bldg_ctrl_pena as bldg_ctrl_pena_15
from models.control.bldg_ctrl_pena_16 import bldg_ctrl_pena as bldg_ctrl_pena_16
from models.control.bldg_ctrl_pena_19 import bldg_ctrl_pena as bldg_ctrl_pena_19
from models.control.bldg_ctrl_pena_20 import bldg_ctrl_pena as bldg_ctrl_pena_20
from models.control.bldg_ctrl_pena_37 import bldg_ctrl_pena as bldg_ctrl_pena_37
from models.control.bldg_ctrl_pena_38 import bldg_ctrl_pena as bldg_ctrl_pena_38
from models.control.bldg_ctrl_pena_39 import bldg_ctrl_pena as bldg_ctrl_pena_39
from models.control.bldg_ctrl_pena_59 import bldg_ctrl_pena as bldg_ctrl_pena_59
from models.control.bldg_ctrl_pena_60 import bldg_ctrl_pena as bldg_ctrl_pena_60
from models.control.bldg_ctrl_pena_61 import bldg_ctrl_pena as bldg_ctrl_pena_61
from models.control.bldg_ctrl_pena_62 import bldg_ctrl_pena as bldg_ctrl_pena_62
from models.control.bldg_ctrl_pena_63 import bldg_ctrl_pena as bldg_ctrl_pena_63
from models.control.bldg_ctrl_pena_67 import bldg_ctrl_pena as bldg_ctrl_pena_67
from models.control.bldg_ctrl_pena_68 import bldg_ctrl_pena as bldg_ctrl_pena_68
from models.control.bldg_ctrl_pena_69 import bldg_ctrl_pena as bldg_ctrl_pena_69
from models.control.bldg_ctrl_pena_70 import bldg_ctrl_pena as bldg_ctrl_pena_70
from models.control.bldg_ctrl_pena_71 import bldg_ctrl_pena as bldg_ctrl_pena_71
from models.control.bldg_ctrl_pena_72 import bldg_ctrl_pena as bldg_ctrl_pena_72
from models.control.bldg_ctrl_pena_74 import bldg_ctrl_pena as bldg_ctrl_pena_74
from models.control.bldg_ctrl_pena_75 import bldg_ctrl_pena as bldg_ctrl_pena_75
from models.control.bldg_ctrl_pena_76 import bldg_ctrl_pena as bldg_ctrl_pena_76
from models.control.bldg_ctrl_pena_77 import bldg_ctrl_pena as bldg_ctrl_pena_77
from models.control.bldg_ctrl_pena_78 import bldg_ctrl_pena as bldg_ctrl_pena_78
from models.control.bldg_ctrl_pena_79 import bldg_ctrl_pena as bldg_ctrl_pena_79
from models.control.bldg_ctrl_pena_81 import bldg_ctrl_pena as bldg_ctrl_pena_81
from models.control.bldg_ctrl_pena_87 import bldg_ctrl_pena as bldg_ctrl_pena_87
from models.control.bldg_ctrl_pena_118 import bldg_ctrl_pena as bldg_ctrl_pena_118
from models.control.bldg_ctrl_pena_131 import bldg_ctrl_pena as bldg_ctrl_pena_131
from models.control.bldg_ctrl_pena_148 import bldg_ctrl_pena as bldg_ctrl_pena_148
from models.control.bldg_pena_36 import bldg_pena_36
from models.control.bldg_pena_51 import bldg_pena_51
from models.control.bldg_pena_52 import bldg_pena_52
from models.control.bldg_pena_73 import bldg_pena_73
from models.control.bldg_grid_agg_data_driven_mdl_large import bldg_grid_agg_data_driven_mdl_large
from models.control.bldg_grid_agg_data_driven_mdl_med import bldg_grid_agg_data_driven_mdl_med
from models.control.bldg_grid_agg_data_driven_mdl_small import bldg_grid_agg_data_driven_mdl_small
from models.simulation.bldg_sim_pena_1 import bldg_sim_pena as bldg_sim_pena_1
from models.simulation.bldg_sim_pena_9 import bldg_sim_pena as bldg_sim_pena_9
from models.simulation.bldg_sim_pena_10 import bldg_sim_pena as bldg_sim_pena_10
from models.simulation.bldg_sim_pena_14 import bldg_sim_pena as bldg_sim_pena_14
from models.simulation.bldg_sim_pena_15 import bldg_sim_pena as bldg_sim_pena_15
from models.simulation.bldg_sim_pena_16 import bldg_sim_pena as bldg_sim_pena_16
from models.simulation.bldg_sim_pena_19 import bldg_sim_pena as bldg_sim_pena_19
from models.simulation.bldg_sim_pena_20 import bldg_sim_pena as bldg_sim_pena_20
from models.simulation.bldg_sim_pena_37 import bldg_sim_pena as bldg_sim_pena_37
from models.simulation.bldg_sim_pena_38 import bldg_sim_pena as bldg_sim_pena_38
from models.simulation.bldg_sim_pena_39 import bldg_sim_pena as bldg_sim_pena_39
from models.simulation.bldg_sim_pena_59 import bldg_sim_pena as bldg_sim_pena_59
from models.simulation.bldg_sim_pena_60 import bldg_sim_pena as bldg_sim_pena_60
from models.simulation.bldg_sim_pena_61 import bldg_sim_pena as bldg_sim_pena_61
from models.simulation.bldg_sim_pena_62 import bldg_sim_pena as bldg_sim_pena_62
from models.simulation.bldg_sim_pena_63 import bldg_sim_pena as bldg_sim_pena_63
from models.simulation.bldg_sim_pena_67 import bldg_sim_pena as bldg_sim_pena_67
from models.simulation.bldg_sim_pena_68 import bldg_sim_pena as bldg_sim_pena_68
from models.simulation.bldg_sim_pena_69 import bldg_sim_pena as bldg_sim_pena_69
from models.simulation.bldg_sim_pena_70 import bldg_sim_pena as bldg_sim_pena_70
from models.simulation.bldg_sim_pena_71 import bldg_sim_pena as bldg_sim_pena_71
from models.simulation.bldg_sim_pena_72 import bldg_sim_pena as bldg_sim_pena_72
from models.simulation.bldg_sim_pena_74 import bldg_sim_pena as bldg_sim_pena_74
from models.simulation.bldg_sim_pena_75 import bldg_sim_pena as bldg_sim_pena_75
from models.simulation.bldg_sim_pena_76 import bldg_sim_pena as bldg_sim_pena_76
from models.simulation.bldg_sim_pena_77 import bldg_sim_pena as bldg_sim_pena_77
from models.simulation.bldg_sim_pena_78 import bldg_sim_pena as bldg_sim_pena_78
from models.simulation.bldg_sim_pena_79 import bldg_sim_pena as bldg_sim_pena_79
from models.simulation.bldg_sim_pena_81 import bldg_sim_pena as bldg_sim_pena_81
from models.simulation.bldg_sim_pena_87 import bldg_sim_pena as bldg_sim_pena_87
from models.simulation.bldg_sim_pena_118 import bldg_sim_pena as bldg_sim_pena_118
from models.simulation.bldg_sim_pena_131 import bldg_sim_pena as bldg_sim_pena_131
from models.simulation.bldg_sim_pena_148 import bldg_sim_pena as bldg_sim_pena_148
from models.simulation.bldg_sim_pena_36 import bldg_sim_pena_36
from models.simulation.bldg_sim_pena_51 import bldg_sim_pena_51
from models.simulation.bldg_sim_pena_52 import bldg_sim_pena_52
from models.simulation.bldg_sim_pena_73 import bldg_sim_pena_73
# from models.simulation.bldg_sim_mdl_bldg62 import bldg_sim_mdl_bldg62
from models.simulation.bldg_sim_mdl_large import bldg_sim_mdl_large
from models.simulation.bldg_sim_mdl_med import bldg_sim_mdl_med
from models.simulation.bldg_sim_mdl_small import bldg_sim_mdl_small
from models.control.grid_aggregator import grid_aggregator

dt_num_offset = 30

start_time = 32*60 - dt_num_offset   # Start time in minutes; 700
dt = 1              # Time-step in minutes

tmp = opt.LCDMPC(start_time, dt)

time = 1*60 + dt_num_offset     # Length of simulation in minutes
horiz_len = 5   # Prediction horizion length
commuincation_iterations = 6 # number of communications between subsystems
Beta = 0.4      # Convex combination parameter for control action

time_array = np.arange(start_time, (start_time + time), dt)

# bldg1_disturb_file = 'input/ROM_simulation_data_interp.csv'
bldg1_small_disturb_file = 'input/ROM_simulation_data_small_office.csv'
bldg1_disturb_file = 'input/ROM_simulation_data_large_office_denver.csv'
# bldg1_disturb_file = 'models/model_identification/building_60_1min.csv'

num_buildings_large = 12
num_buildings_medium = 0
num_buildings_small = 0
num_buildings_total = num_buildings_large + num_buildings_medium + num_buildings_small

ms_dot_large = 8.0
T_sa_large = 12.8
T_oa_large = 28.
T_z_large = 23.0 # 21.2
T_e_large = 20.

ms_dot_large2 = 6.0
T_sa_large2 = 12.8
T_oa_large2 = 28.
T_z_large2 = 23.0 # 22.0
T_e_large2 = 20.

ms_dot_large3 = 10.0
T_sa_large3 = 12.8
T_oa_large3 = 28.
T_z_large3 = 23.0 # 22.3
T_e_large3 = 20.

ms_dot_large4 = 10.0
T_sa_large4 = 12.8
T_oa_large4 = 28.
T_z_large4 = 23.0 # 22.3
T_e_large4 = 20.

ms_dot_large5 = 10.0
T_sa_large5 = 12.8
T_oa_large5 = 28.
T_z_large5 = 23.0 # 22.3
T_e_large5 = 20.

ms_dot_large6 = 10.0
T_sa_large6 = 12.8
T_oa_large6 = 28.
T_z_large6 = 23.0 # 22.3
T_e_large6 = 20.

ms_dot_medium = 8.0
T_sa_medium = 12.8
T_oa_medium = 28.
T_z_medium = 23.0
T_e_medium = 20.

# ms_dot_medium = 1.0
# T_sa_medium = 12.8
# T_oa_medium = 28.95
# T_z_medium = 22.0
# T_e_medium = 20.

ms_dot_small = 1.0
T_sa_small = 12.8
T_oa_small = 28.
T_z_small = 24.3313
T_e_small = 24.

Q_int = 8.0
Q_solar = 15.0

inputs = [ms_dot_large, T_sa_large, T_oa_large, T_z_large]
inputs_large = [ms_dot_large, T_sa_large, T_oa_large, T_z_large]
inputs_large2 = [ms_dot_large2, T_sa_large2, T_oa_large2, T_z_large2]
inputs_large3 = [ms_dot_large3, T_sa_large3, T_oa_large3, T_z_large3]
inputs_large4 = [ms_dot_large4, T_sa_large4, T_oa_large4, T_z_large4]
inputs_large5 = [ms_dot_large5, T_sa_large5, T_oa_large5, T_z_large5]
inputs_large6 = [ms_dot_large6, T_sa_large6, T_oa_large6, T_z_large6]
inputs_medium = [ms_dot_medium, T_sa_medium, T_oa_medium, T_z_medium]
inputs_small = [ms_dot_small, T_sa_small, T_oa_small, T_z_small]

inputs_large_all = [inputs_large, inputs_large2, inputs_large3, inputs_large4, inputs_large5, inputs_large6]
ms_dot_large_all = [ms_dot_large, ms_dot_large2, ms_dot_large3, ms_dot_large4, ms_dot_large5, ms_dot_large6]
T_sa_large_all = [T_sa_large, T_sa_large2, T_sa_large3, T_sa_large, T_sa_large, T_sa_large]
T_z_large_all = [T_z_large, T_z_large2, T_z_large3, T_z_large, T_z_large, T_z_large]
T_e_large_all = [T_e_large, T_e_large2, T_e_large3, T_e_large, T_e_large, T_e_large]

disturb1 = [6.0, 2.0, 2.0]
disturb2 = [6.0, 2.0, 2.0]

outputs1 = [1]
outputs2 = [1]

# refs_large = [[0.0], [20.], [0.0]]
# refs_large = [[21.5 - 22.794], [20.], [0.0]]
# refs_large2 = [[21.5 - 22.794], [20.], [0.0]]
# refs_large3 = [[21.5 - 22.794], [20.], [0.0]]
refs_large = [[21.65 - 21], [20.], [0.0], [0.0]]
refs_large2 = [[21.5 - 23.93], [20.], [0.0], [0.0]]
refs_large3 = [[21.0 - 23.91], [20.], [0.0], [0.0]]
refs_large4 = [[21.0 - 23.88], [20.], [0.0], [0.0]]
refs_large5 = [[21.0 - 20.35], [20.], [0.0], [0.0]]
refs_large6 = [[21.0 - 23.78], [20.], [0.0], [0.0]]
# refs_60 = [[21.5 - 20.853], [20.], [0.0]]
# refs_62 = [[21.5 - 20.853], [20.], [0.0]]
# refs_large_all = [refs_60, refs_62]
refs_large_all = [refs_large, refs_large2, refs_large3, refs_large4, refs_large5, refs_large6]


refs_medium = [[21.5 - 24.66], [20.], [0.0], [0.0]]
refs_small = [[0], [20.], [0.0]]

disturbance_data = pd.read_csv(bldg1_disturb_file)
Toa_horiz = disturbance_data.iloc[start_time: start_time + int(time/dt) + horiz_len]['T_outdoor'].values
Toa_horiz_normed = Toa_horiz/Toa_horiz[0]

np.random.seed(1)
# grid_agg_ref = np.random.normal(6*num_buildings_small + 20*num_buildings_medium + 10*num_buildings_large, 0.0, int(time/dt) + horiz_len)*Toa_horiz_normed
grid_agg_ref = np.random.normal(6*num_buildings_small + 10*num_buildings_medium + 7*num_buildings_large, 2.0, int(10*60/dt) + horiz_len)#*Toa_horiz_normed

refs_grid_total = pd.DataFrame()
for i in range(int(time/dt) + horiz_len):
    refs_grid_total = refs_grid_total.append(
        {
            'time': start_time + i,
            'grid_ref': [[grid_agg_ref[i]]] + [[0.] for i in range(int(num_buildings_total / 2))]
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
bldg_optoptions_small = {
    # 'Major feasibility tolerance': 1e1,
    'Print file': 'SNOPT_bldg_small_print.out',
    'Summary file': 'SNOPT_bldg_small_summary.out',
    'Proximal iterations limit': 1000,
}
grid_optoptions = {
    # 'Major feasibility tolerance': 1e3,
    'Print file': 'SNOPT_grid_print.out',
    'Summary file': 'SNOPT_grid_summary.out',
    'Proximal iterations limit': 1000,
}

# num_downstream1 = num_buildings_total
# num_upstream1 = 0

num_downstream1 = 6
num_upstream1 = 0
num_downstream2 = 6
num_upstream2 = 0

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

building_ids = [20, 37, 38, 39, 59, 60, 61, 62, 72, 79, 81, 87]
i = 0
for j in range(len(building_ids)):
    str1 = 'building_control_models.append(bldg_ctrl_pena_' + str(building_ids[j]) + '(ms_dot_large_all[i], T_sa_large_all[i], T_z_large_all[i], horiz_len, energy_red_weight[i], Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i], Qint_offset[i], Qsol_offset[i]))'
    eval(str1)
    # building_control_models.append(
    #     bldg_ctrl_pena_148(
    #         ms_dot_large_all[i], T_sa_large_all[i], T_z_large_all[i], horiz_len, energy_red_weight[i],
    #         Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
    #         Qint_offset[i], Qsol_offset[i]
    #     )
    # )
    str2 = 'building_truth_models.append(bldg_sim_pena_' + str(building_ids[j]) + '(dt/60, ms_dot_large_all[i], T_sa_large_all[i], T_z_large_all[i], T_e_large_all[i], start_time, Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],  Qint_offset[i], Qsol_offset[i]))'
    eval(str2)
    # building_truth_models.append(
    #     bldg_sim_pena_148(dt/60, ms_dot_large_all[i], T_sa_large_all[i], T_z_large_all[i], T_e_large_all[i], start_time,
    #     Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
    #     Qint_offset[i], Qsol_offset[i])
    # )

# for i in range(num_buildings_large):
#     building_control_models.append(
#         bldg_ctrl_pena_39(
#             ms_dot_large_all[i], T_sa_large_all[i], T_z_large_all[i], horiz_len, energy_red_weight[i],
#             Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
#             Qint_offset[i], Qsol_offset[i]
#         )
#     )
#     building_truth_models.append(
#         bldg_sim_pena_39(dt/60, ms_dot_large_all[i], T_sa_large_all[i], T_z_large_all[i], T_e_large_all[i], start_time,
#         Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
#         Qint_offset[i], Qsol_offset[i])
#     )

for i in range(num_buildings_medium):
    building_control_models.append(
        bldg_pena_73(
            ms_dot_medium, T_sa_medium, T_z_medium, horiz_len, energy_red_weight[i],
            Qint_std[i], Qsol_std[i], Qint_scale[i], Qsol_scale[i],
            Qint_offset[i], Qsol_offset[i]
        )
    )
    building_truth_models.append(
        bldg_sim_pena_73(dt/60, ms_dot_medium, T_sa_medium, T_z_medium, T_e_medium, start_time,
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

grid_agg2_cont = grid_aggregator(horiz_len, num_downstream2)
grid_agg2_truth = grid_aggregator(horiz_len, num_downstream2)

tmp.build_subsystem(0, grid_agg1_cont, grid_agg1_truth, 
    inputs, outputs1, horiz_len, Beta, bldg1_disturb_file, refs_total=refs_grid_total,
    optOptions=grid_optoptions)

tmp.build_subsystem(1, grid_agg2_cont, grid_agg2_truth, 
    inputs, outputs1, horiz_len, Beta, bldg1_disturb_file, refs_total=refs_grid_total,
    optOptions=grid_optoptions)

for i in range(num_buildings_large):
    # print('i large: ', i + 1)
    j=0
    tmp.build_subsystem(i + 2, building_control_models[i], building_truth_models[i],
    inputs_large_all[j], outputs1, horiz_len, Beta, bldg1_disturb_file, refs=refs_large_all[j],
    optOptions=bldg_optoptions_large)

for i in range(num_buildings_medium):
    # print('i medium: ', i + 1 + num_buildings_large)
    ind = i + 1 + num_buildings_large
    tmp.build_subsystem(ind, building_control_models[ind-1], building_truth_models[ind-1],
    inputs_medium, outputs1, horiz_len, Beta, bldg1_disturb_file, refs=refs_medium,
    optOptions=bldg_optoptions_med)

for i in range(num_buildings_small):
    # print('i small: ', i + 1 + num_buildings_large + num_buildings_medium)
    ind = i + 1 + num_buildings_large + num_buildings_medium
    tmp.build_subsystem(ind, building_control_models[ind-1], building_truth_models[ind-1],
    inputs_small, outputs1, horiz_len, Beta, bldg1_small_disturb_file, refs=refs_small,
    optOptions=bldg_optoptions_small)

# connections = [[0, i+1] for i in range(num_buildings_total)] + \
#               [[i+1, 0] for i in range(num_buildings_total)]

connections = [[0, i+2] for i in range(num_downstream1)] + \
              [[i+2, 0] for i in range(num_downstream1)] + \
              [[1, i + num_downstream1 + 2] for i in range(num_downstream2)] + \
              [[i + num_downstream1 + 2, 1] for i in range(num_downstream2)]

# connections = None
tmp.build_interconnections(interconnections=connections)

outputs_all = []
disturbance_all = []
controls_all = []
gamma_all = []

# tmp.calc_stability()
# lkj

# for i in range(int(time/dt)):

#     print('+++++++++++++++++++++++++++++')
#     print('time iteration: ', i)
#     print('+++++++++++++++++++++++++++++')

#     # TODO: Need to map states to updated state (LPV like)

#     tmp.relinearize_subsystem_models()

#     gamma_comm = []
#     for j in range(commuincation_iterations):
#         # Communication step
#         tmp.communicate()
#         # Optimize all subsystems (individual objective functions)
#         tmp.optimize_all()
#         # Convex summation of control parameters (for stability)
#         tmp.convex_sum_cont()
#         # Update Z's for downstream subsystems
#         tmp.update_downstream_outputs()

#         gamma_comm.append(tmp.calculate_sensitivities())

#         # print('==============================')
#         print('communication iteration: ', j)
#         # print('==============================')
#     # Update state equations for subsystems
#     tmp.update_states()
#     # Update outputs of subsystems
#     tmp.update_subsystem_outputs()

#     # Gather outputs for plotting purposes
#     outputs = tmp.simulate_truth_model()
#     outputs_all.append(outputs)

#     # Update control model's filter
#     tmp.update_control_filter()
#     # Update values for linearization
#     tmp.update_inputs_for_linearization()
#     # Get updated forecast inputs
#     tmp.update_forecast_inputs()
#     # Gather control actions for plotting purposes
#     disturbance_all.append([[subsys.d] for subsys in tmp.subsystems])
#     controls_all.append([[subsys.uConv] for subsys in tmp.subsystems])
#     gamma_all.append(gamma_comm)

import pickle

# pickle.dump([tmp, outputs_all, disturbance_all, controls_all, gamma_all], open('results.p', 'wb'))

tmp, outputs_all, disturbance_all, controls_all, gamma_all = pickle.load(
    open('results.p', 'rb')
)

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

plt.savefig('gammas.pdf', bbox_inches='tight')


np.set_printoptions(suppress=True)

# gamma_plot = []
# for i in range(commuincation_iterations):
#     gamma_plot.append()

plot_temps = []
for i in range(num_buildings_total):
    plot_temps.append([val[i+2:][0][0][0][0] for val in outputs_all])

plot_bldg_powers = []
for i in range(num_buildings_total):
    plot_bldg_powers.append([val[i+2:][0][1][0][0] for val in outputs_all])

# print(outputs_all)
# print('-------------')
# print(outputs_all[0][0])
# print(outputs_all[0][0][0])

grid_prefs_ind = []
for i in range(int(num_buildings_total / 2)):
    grid_prefs_ind.append([val[0][i][0] for val in outputs_all])
    grid_prefs_ind.append([val[1][i][0] for val in outputs_all])

# print(grid_prefs_ind)
# lkj

total_power = []
grid_prefs_total = []

for i in range(len(outputs_all)):
    sum_of_powers = np.sum([val[1] for val in outputs_all[i][2:]])
    sum_of_grid_prefs = np.sum(outputs_all[i][0][:])
    total_power.append(sum_of_powers)
    grid_prefs_total.append(sum_of_grid_prefs)

T_lower = 20.0
T_upper = 23.3
P_lower = 0.0
P_upper = 100.0

fig, axes = plt.subplots(2, 2, figsize=(16,10))

disturbance_data = pd.read_csv(bldg1_disturb_file)

colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan', 'tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
# colors = ['tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:blue', 'tab:red']
ax2 = axes[0, 0]
bldg_temp_legend = ['Large Office Building 0']
for i in range(num_buildings_total):
    ax2.plot(
        time_array[dt_num_offset:], np.array(plot_temps[i]).flatten()[dt_num_offset:],
        color=colors[i]
    )
    # bldg_temp_legend.append('Large Office Building ' + str(i))
ax2.plot(
    time_array[dt_num_offset:],
    disturbance_data.iloc[
        start_time + dt_num_offset: start_time + int(time/dt)
    ]['T_outdoor'].values, color='gray'
)
ax2.plot(
    time_array[dt_num_offset:], T_lower*np.ones(len(time_array))[dt_num_offset:], '--k'
)
ax2.plot(
    time_array[dt_num_offset:], T_upper*np.ones(len(time_array))[dt_num_offset:], '--k'
)
ax2.plot(time_array[dt_num_offset:], (np.array(refs_large[0]) + 22.794)*np.ones(len(time_array))[dt_num_offset:], '--r')
ax2.legend(bldg_temp_legend + ['Outdoor Air Temp'])
xticks = ax2.get_xticks()
ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax2.set_title('Building Temperatures')
ax2.set_ylabel('Temperature [C]')
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
bldg_power_legend = ['Large Office Building', 'Medium Office Building 1', 'Medium Office Building 2']
for i in range(num_buildings_total):
    ax2.plot(
        time_array[dt_num_offset:],
        np.array(plot_bldg_powers[i]).flatten()[dt_num_offset:],
        color=colors[i]
    )
    # bldg_power_legend.append('Large Office Building ' + str(i))
for i in range(num_buildings_total):
    ax2.plot(
        time_array[dt_num_offset:],
        np.array(grid_prefs_ind[i]).flatten()[dt_num_offset:],
        '--',
        color=colors[i]
    )
    bldg_power_legend.append(r'$P_{ref}$ Building ' + str(i))
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

plt.savefig('results.pdf', bbox_inches='tight')

plt.show()


