# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not 
# use this file except in compliance with the License. You may obtain a copy of 
# the License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT 
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the 
# license for the specific language governing permissions and limitations under 
# the License.

class LCDMPC():
    def __init__(self):
        self.subsystems = []

    def build_subsystem(self, A, B, C, D, inputs, outputs, ID=None):
        # create subsystem object
        subsys = subsystem(self, A, B, C, D, inputs, outputs, ID)
        # append it to subsystem list
        self.subsystems.append(subsys)
        
    def sim_control(self):
        # call optimize and calculate horizon
        pass

    def calculate_horizon(self):
        pass

    def optimize(self):
        pass

    def reinitialize(self):
        pass

    def sim_system(self):
        # simulate the real-world model
        pass

    def build_interconnections(self):
        # accept inteconnections from subsystems and setup 
        # interconnection matrix
        pass

    def set_opt_bounds(self):
        pass

class subsystem():
    def __init__(self, obj, A, B, C, D, inputs, outputs, upstream=None, downstream=None, ID=None):
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.inputs = inputs
        self.outputs = outputs
        if upstream is not None:
            self.upstream = upstream
        else:
            self.upstream = []
        if downstream is not None:
            self.downstream = downstream
        else:
            self.downstream = []
        if ID is not None:
            self.ID = ID
        else:
            self.ID = str(len(obj.subsystems) + 1)

    def optimize():
        pass