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

tmp = opt.LCDMPC()

horiz_len = 5

# A = np.mat([[1, 0], 
#             [0, 2]])
# Bu = np.mat([[2, 0], 
#              [0, 1]])
# Bv = np.mat([[0, 0], 
#              [1, 0]])
# Bd = np.zeros((2, 2))
# Cy = np.mat([0, 5])
# Cz = np.zeros((1, 2))
# Dy = np.mat([0, 2])
# Dz = np.zeros((1, 2))

A = np.array([[2, 4], 
            [6, 8]])
Bu = np.array([[1, 1], 
             [1, 1]])
Bv = np.array([[0, 0], 
             [1, 0]])
Bd = np.array([[2], [2]])
Cy = np.array([1, 1])
# Cy = np.array([[1, 1], [0, 1]])
# Cz = np.array([2, 2])
Cz = np.array([[2, 2], [1, 1]])
Dy = np.array([[0, 2], [0, 0]])
Dz = np.array([[1, 2], [1, 0]])

inputs = [1,2,3]
outputs = [4,5]
ID = 'building 1'

tmp.build_subsystem(A, Bu, Bv, Bd, Cy, Cz, Dy, Dz, inputs, outputs, horiz_len)
tmp.build_subsystem(A, Bu, Bv, Bd, Cy, Cz, Dy, Dz, inputs, outputs, horiz_len)

tmp.subsystems[0].sys_matrices()

np.set_printoptions(suppress=True)

# print(np.array(tmp.subsystems[0].Fy))
# print(np.array(tmp.subsystems[0].Fz))
print(tmp.subsystems[0].Nz)

# print(tmp.subsystems[0].nodeID)
# print(tmp.subsystems[1].nodeID)

# print(tmp.subsystems[0].nodeName)
# print(tmp.subsystems[1].nodeName)