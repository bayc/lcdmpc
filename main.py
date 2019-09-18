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

tmp = opt.LCDMPC()

time = 120       # Length of simulation in minutes
dt = 5          # Time-step in minutes
horiz_len = 5   # Prediction horizion length
commuincation_iterations = 3
Beta = 0.5      # Convex combination parameter for control action

outputs1 = [1]
outputs2 = [1]

refs1 = [-1]
refs2 = [-5]

t_low = 21
t_upper = 27

cons1 = {'name':'con1', 'ncon':1, 'lower':[t_low], 
         'upper':[t_upper], 'wrt':['U']}

ms_dot = 0.8
T_sa = 12.8
T_oa = 6.
T_z = 1.

inputs = [ms_dot, T_sa, T_oa, T_z]

bldg1 = bldg_data_driven_mdl(ms_dot, T_sa, T_oa, T_z)
bldg2 = bldg_data_driven_mdl(ms_dot, T_sa, T_oa, T_z)

tmp.build_subsystem(bldg1, 
    inputs, outputs1, refs1, horiz_len, Beta, cons1)

tmp.build_subsystem(bldg2, 
    inputs, outputs2, refs1, horiz_len, Beta, cons1)

# tmp.build_subsystem(A, Bu, Bv, Bd, Cy, Cz, Dyu, Dyv, Dzu, Dzv, 
#     inputs, outputs1, refs1, horiz_len, Beta, cons1)
# tmp.build_subsystem(A*2, Bu*0.2, Bv, Bd, Cy, Cz, Dyu, Dyv, Dzu, Dzv, 
#     inputs, outputs2, refs2, horiz_len, Beta, cons1)

# tmp.subsystems[0].sys_matrices()

connections = [[0, 1], [1, 0]]
# connections = [[1, 0]]

tmp.build_interconnections(interconnections=connections)

cont1 = []
out1 = []

for i in range(int(time/dt)):

    # TODO: add Rohit's filter update

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

    cont1.append(tmp.subsystems[0].uOpt[0:3])
    out1.append(tmp.subsystems[0].y)

    print('+++++++++++++++++++++++++++++')
    print('time iteration: ', i)
    print('+++++++++++++++++++++++++++++')

print(cont1)
print(out1)

np.set_printoptions(suppress=True)

# tmp.update_outputs()
# tmp.communicate()

# print(tmp.subsystems[0].v)
# print(tmp.subsystems[1].v)

# print(tmp.subsystems[0].nodeID)
# print(tmp.subsystems[0].downstream)
# print(tmp.subsystems[0].upstream)

# print(tmp.subsystems[1].nodeID)
# print(tmp.subsystems[1].downstream)
# print(tmp.subsystems[1].upstream)

# print(np.array(tmp.subsystems[0].Fy))
# print(np.array(tmp.subsystems[0].Fz))
# print(tmp.subsystems[0].My)

# print(tmp.subsystems[0].nodeID)
# print(tmp.subsystems[1].nodeID)

# print(tmp.subsystems[0].nodeName)
# print(tmp.subsystems[1].nodeName)