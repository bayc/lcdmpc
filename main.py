# Copyright 2019 NREL

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
# import numpy.linalg.matrix_power as matpow

from bldg_data_driven_mdl import bldg_data_driven_mdl
from wind_mdl import wind_mdl
from floris.floris import Floris

tmp = opt.LCDMPC()

time = 240       # Length of simulation in minutes
dt = 5          # Time-step in minutes
horiz_len = 5   # Prediction horizion length
commuincation_iterations = 1
Beta = 0.0      # Convex combination parameter for control action

ms_dot = 0.8
T_sa = 12.8
T_oa = 6.
T_z = 1.

inputs = [ms_dot, T_sa, T_oa, T_z]

disturb1 = [6.0, 2.0, 2.0]
disturb2 = [6.0, 2.0, 2.0]

outputs1 = [1]
outputs2 = [1]

refs1 = [-2]
refs2 = [-1]

bldg1 = bldg_data_driven_mdl(ms_dot, T_sa, T_oa, T_z, horiz_len)
bldg2 = bldg_data_driven_mdl(ms_dot, T_sa, T_oa, T_z, horiz_len)

input_file = 'example_input.json'

yaw = [0.5, 0.5]
Ct = [0.5, 0.5]
ws = 8.
wd = 359.

wrefs1 = [-20000]
wBeta = 0.0
winputs1 = [yaw, Ct, ws, wd]
wdisturb1 = [8., 359.]
woutputs1 = [4.27]

wind1 = wind_mdl(input_file, yaw, Ct, ws, wd, horiz_len)

# tmp.build_subsystem(bldg1, 
#     inputs, disturb1, outputs1, refs1, horiz_len, Beta)

# tmp.build_subsystem(bldg2, 
#     inputs, disturb2, outputs2, refs2, horiz_len, Beta)

tmp.build_subsystem(wind1, winputs1, wdisturb1, woutputs1, 
                    wrefs1, horiz_len, wBeta)
# tmp.build_subsystem(wind1, winputs, woutputs2, wrefs2, horiz_len, wBeta)

# tmp.build_subsystem(A, Bu, Bv, Bd, Cy, Cz, Dyu, Dyv, Dzu, Dzv, 
#     inputs, outputs1, refs1, horiz_len, Beta, cons1)
# tmp.build_subsystem(A*2, Bu*0.2, Bv, Bd, Cy, Cz, Dyu, Dyv, Dzu, Dzv, 
#     inputs, outputs2, refs2, horiz_len, Beta, cons1)

# tmp.subsystems[0].sys_matrices()

connections = [[0, 1], [1, 0]]
# connections = [[1, 0]]

# tmp.build_interconnections(interconnections=connections)
tmp.build_interconnections(interconnections=None)

cont1 = []
out1 = []
cont2 = []
out2 = []

for i in range(int(time/dt)):

    # TODO: add Rohit's filter update
    # TODO: Need to map states to updated state (LPV like)

    tmp.relinearize_subsystem_models()

    for j in range(commuincation_iterations):
        tmp.communicate()       # Communication step

        tmp.optimize_all()

        tmp.convex_sum_cont()

        tmp.update_downstream_outputs()

        tmp.calculate_sensitivities()

        print('==============================')
        print('communication iteration: ', j)
        print('==============================')

    tmp.update_states()

    tmp.update_subsystem_outputs()

    tmp.update_inputs_for_linearization()

    # cont1.append(tmp.subsystems[0].uConv[0:2*len(yaw)])
    cont1.append(tmp.subsystems[0].uConv)
    out1.append(tmp.subsystems[0].y)

    # cont2.append(tmp.subsystems[1].uOpt[0:3])
    # out2.append(tmp.subsystems[1].y)

    print('+++++++++++++++++++++++++++++')
    print('time iteration: ', i)
    print('+++++++++++++++++++++++++++++')

print(cont1)
print(out1)
# print(cont2)
# print(out2)

np.set_printoptions(suppress=True)

