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

tmp = opt.LCDMPC()

A = 1
B = 2
C = 3
D = 4

inputs = [1,2,3]
outputs = [4,5]
ID = 'building 1'

tmp.build_subsystem(A, B, C, D, inputs, outputs, ID=ID)
tmp.build_subsystem(A, B, C, D, inputs, outputs, ID='building 2')

print(tmp.subsystems[0].ID)
print(tmp.subsystems[1].ID)