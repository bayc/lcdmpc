# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not 
# use this file except in compliance with the License. You may obtain a copy of 
# the License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT 
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the 
# license for the specific language governing permissions and limitations under 
# the License.

import numpy as np
from numpy import dot as dot
from numpy import transpose as tp
from numpy.linalg import matrix_power as matpower

class LCDMPC():
    def __init__(self):
        self.subsystems = []

    def build_subsystem(self, A, Bu, Bv, Bd, Cy, Cz, Dy, Dz, inputs, outputs, 
                        horiz_len, nodeID=None, nodeName=None):
        # create subsystem object
        subsys = subsystem(self, A, Bu, Bv, Bd, Cy, Cz, Dy, Dz, inputs, \
            outputs, horiz_len, nodeID=nodeID, nodeName=nodeName)
        # append it to subsystem list
        self.subsystems.append(subsys)
        
    def sim_control(self):
        # call optimize and calculate horizon
        pass

    def optimize(self):
        pass

    def reinitialize(self):
        pass

    def sim_system(self):
        # simulate the real-world model
        pass

    def build_interconnections(self, interconnections):
        """
        Accept inteconnections from subsystems and setup 
        interconnection. matrix
        
        Args:
            interconnections (list): A list of tuples containing the 
                upstream and downstream nodeIds.
        """
        for pair in interconnections:
            up = pair[0]
            down = pair[1]

            self.subsystems[up].downstream.append(down)
            self.subsystems[down].upstream.append(up)

    def update_outputs(self):
        """
        Update the z vectors of the subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_z()

    def communicate(self):
        """
        Exchange information between subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_v(self)

class subsystem():
    def __init__(self, obj, A, Bu, Bv, Bd, Cy, Cz, Dy, Dz,K,mean_inputs,mean_output,x_k_init, 
                 inputs, outputs, horiz_len, upstream=None, 
                 downstream=None, nodeID=None, nodeName=None):
        self.A = A
        self.Bu = Bu
        self.Bv = Bv
        self.Bd = Bd
        self.Cy = Cy
        self.Cz = Cz
        self.Dy = Dy
        self.Dz = Dz
        self.K = K
        self.x_k_k = x_k_init
        self.inputs = inputs
        self.outputs = outputs
        self.mean_inputs = mean_inputs # State space model developed with means removed
        self.mean_output = mean_output # State space output computed with mean removed
        self.horiz_len = horiz_len
        self.T_pred = [] #Prediction horizon history
        if upstream is not None:
            self.upstream = upstream
        else:
            self.upstream = []
        if downstream is not None:
            self.downstream = downstream
        else:
            self.downstream = []
        if nodeID is not None:
            self.nodeID = nodeID
        else:
            self.nodeID = str(len(obj.subsystems))
        if nodeName is not None:
            self.nodeName = nodeName
        else:
            self.nodeName = 'Node ' + str(len(obj.subsystems))

        self.mat_sizes()
        self.x = []
        self.u = []
        self.v = []
        self.y = []
        self.z = []
        
    def create_mpc_inputs(horizon_inputs):
        mpc_inputs_list = []
        for i in np.arange(self.horiz_len):
            ms_dot = horizon_inputs[0][i]
            T_da = horizon_inputs[1][i]
            Q_hvac = ms_dot
            mpc_inputs_list.append([]) 

    def filter_update(self,applied_inputs = [[0,0],[0,0,0]],measured_output = 21, past_output =21):
        # update subsystem inputs and outputs (last applied and last measured)
        self.inputs = applied_inputs
        self.outputs = measured_output
        
        # Calculate Q_hvac for the model
        ms_dot = applied_inputs[0][0]
        T_da = applied_inputs[0][1]
        Q_hvac = ms_dot*(T_da - measured_output)
        Q_hvac_dz = Q_hvac - self.mean_inputs[-1]
        # State space inputs
        d_dz = np.subtract(applied_inputs[1],mean_inputs[:-1]) # Disturbance inputs with mean removed
        ss_cntrl_inputs = np.array([Q_hvac_dz])
        ss_disturbance_inputs = d_dz
        ss_disturbance_inputs[0] = applied_inputs[0][0] - past_output
        x_k_k1 = self.A*self.x_k_k + np.sum(self.Bu*ss_cntrl_inputs)+np.sum(self.Bd*ss_disturbance_inputs)
        # State update
        self.x_k_k = self.C*x_k_k1 + self.K*(measured_output- self.mean_output - self.C*x_k_k1) 
    
    def predicted_horizon(self,applied_inputs = [[0,0],[0,0,0]], measured_output = 21, past_output,horizon_inputs):
        self.filter_update(applied_inputs,measured_output,past_output)
        T_horizon = []
        for i in arange(np.size(mpc_inputs,0)):
            ss_disturbance_inputs = [horizon_inputs[j][i] for j in range(2,5)]
            
            if i == 0: 
                ss_disturbance_inputs[0] = horizon_inputs[2][i] - measured_output
                ss_cntrl_inputs = horizon_inputs[0][i]*(horizon_inputs[1][i] - measured_output)
            else:
                ss_disturbance_inputs[0] = horizon_inputs[2][i] - T_horizon[i-1]
                ss_cntrl_inputs = horizon_inputs[0][i]*(horizon_inputs[1][i] - T_horizon[i-1])
            ss_disturbance_inputs = np.array([ss_cntrl_inputs - self.mean_inputs[:-1]])
            ss_cntrl_inputs = np.array([mpc_inputs[i][-1] - self.mean_inputs[-1]])
            x_k_hat = self.A*self.x_k_k + np.sum(self.Bu*ss_cntrl_inputs)+np.sum(self.Bd*ss_disturbance_inputs)
            T_horizon.append(self.C*x_k_hat + self.mean_output)
            
        self.T_pred.append(T_horizon)

    def set_opt_bounds(self, low, high):
        pass

    def optimize(self):
        pass

    def obj_func(self, control):
        # U^T*H*U + 2*U^T*F + V^T*E*V + 2*V^T*T
        self.update()
        self.U = control
        return dot(dot(tp(self.U), self.H), self.U) + 2*dot(tp(self.U), F) \
            + dot(dot(tp(self.V), self.E), self.V) + 2*dot(tp(self.V), self.T)

    def update(self):
        self.H = dot(dot(tp(self.My), self.Q), self.My) + self.S
        self.E_1 = self.E
        self.E = dot(dot(tp(self.Ny), self.Q), self.Ny)
        self.F = dot(dot(tp(self.My), self.Q), (dot(self.Fy, self.x0) \
               + dot(self.Ny, self.V) - self.ref)) + 0.5*dot(tp(self.Mz), self.psi)
        self.T = dot(dot(tp(self.Ny), self.Q), (dot(self.Fy, self.x0) - self.ref)) \
               + 0.5*dot(tp(self.Nz), self.psi)

    def calc_sens(self):
        self.gamma = 2*(dot(self.E_1, self.V) + self.T \
                   + dot(dot(dot(tp(self.Ny), self.Q), self.My), self.U))

    def update_subsys(self):
        self.update_x()
        self.update_y()
        self.update_z()

    def update_v(self, obj):
        for upstream in self.upstream:
            self.v = obj.subsystems[upstream].z

    def update_x(self):
        self.x = dot(self.A, self.x) + dot(self.Bu, self.u) \
            + dot(self.Bv, self.v) + dot(self.Bd, self.d)

    def update_y(self):
        self.y = dot(self.Cy, self.x) + dot(self.Dy, self.u)

    def update_z(self):
        self.z = dot(self.Fz, self.x) + dot(self.Mz, self.u) \
            + dot(self.Nz, self.v)

    def sys_matrices(self):       
        self.Fy = np.array([dot(self.Cy, matpower(self.A, i)) \
                  for i in range(1, self.horiz_len + 1)])
        if self.Fy[0].ndim > 1:
            self.Fy = np.concatenate(self.Fy, axis=1)
        else:
            self.Fy = np.concatenate(self.Fy)

        self.Fz = np.array([dot(self.Cz, matpower(self.A, i)) \
                  for i in range(0, self.horiz_len)])
        if self.Fz[0].ndim > 1:
            self.Fz = np.concatenate(self.Fz, axis=1)
        else:
            self.Fz = np.concatenate(self.Fz)

        Mytmp = dot(self.Cy, self.Bu)
        MytmpShape = np.shape(Mytmp)
        for i in range(self.horiz_len - 1):
            Mytmp = np.hstack((Mytmp, np.zeros(MytmpShape)))
        for i in range(1, self.horiz_len):
            if Mytmp.ndim == 1:
                Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
                    dot(matpower(self.A, i), self.Bu )), Mytmp[:-self.nyBu]))))
            else:
                if dot(self.Cy, dot(matpower(self.A, i), self.Bu )).ndim == 1:
                    Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
                        dot(matpower(self.A, i), self.Bu )), Mytmp[-self.nxCy:,:-self.nyBu][0]))))
                else:
                    Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
                        dot(matpower(self.A, i), self.Bu )), Mytmp[-self.nxCy:,:-self.nyBu]))))
        self.My = Mytmp

        Mztmp0 = self.Dz
        Mztmp0Shape = np.shape(Mztmp0)
        Mztmp = np.hstack((dot(self.Cz, self.Bu), Mztmp0))
        for i in range(self.horiz_len - 1):
            Mztmp0 = np.hstack((Mztmp0, np.zeros(Mztmp0Shape)))
        for i in range(self.horiz_len - 2):
            Mztmp = np.hstack((Mztmp, np.zeros(Mztmp0Shape)))
        Mztmp = np.vstack((Mztmp0, Mztmp))
        for i in range(1, self.horiz_len - 1):
            if Mztmp.ndim == 1:
                Mztmp = np.vstack((Mztmp, np.hstack((dot(self.Cz, \
                    dot(matpower(self.A, i), self.Bu )), Mztmp[:-self.nyBu]))))
            else:
                if dot(self.Cz, dot(matpower(self.A, i), self.Bu )).ndim == 1:
                    Mztmp = np.vstack((Mztmp, np.hstack((dot(self.Cz, \
                        dot(matpower(self.A, i), self.Bu )), Mztmp[-self.nxCz:,:-self.nyBu][0]))))
                else:
                    Mztmp = np.vstack((Mztmp, np.hstack((dot(self.Cz, \
                        dot(matpower(self.A, i), self.Bu )), Mztmp[-self.nxCz:,:-self.nyBu]))))
        self.Mz = Mztmp
        
        Nytmp = dot(self.Cy, self.Bv)
        NytmpShape = np.shape(Nytmp)
        for i in range(self.horiz_len - 1):
            Nytmp = np.hstack((Nytmp, np.zeros(NytmpShape)))
        for i in range(1, self.horiz_len):
            if Nytmp.ndim == 1:
                Nytmp = np.vstack((Nytmp, np.hstack((dot(self.Cy, \
                    dot(matpower(self.A, i), self.Bv )), Nytmp[:-self.nyBv]))))
            else:
                if dot(self.Cy, dot(matpower(self.A, i), self.Bv )).ndim == 1:
                    Nytmp = np.vstack((Nytmp, np.hstack((dot(self.Cy, \
                        dot(matpower(self.A, i), self.Bv )), Nytmp[-self.nxCy:,:-self.nyBv][0]))))
                else:
                    Nytmp = np.vstack((Nytmp, np.hstack((dot(self.Cy, \
                        dot(matpower(self.A, i), self.Bv )), Nytmp[-self.nxCy:,:-self.nyBv]))))
        self.Ny = Nytmp
        
        Nztmp = dot(self.Cz, self.Bv)
        NztmpShape = np.shape(Nztmp)
        for i in range(self.horiz_len - 1):
            Nztmp = np.hstack((Nztmp, np.zeros(NztmpShape)))
        Nztmp0 = np.zeros(np.shape(Nztmp))
        Nztmp = np.vstack((Nztmp0, Nztmp))
        for i in range(1, self.horiz_len - 1):
            if Nztmp.ndim == 1:
                Nztmp = np.vstack((Nztmp, np.hstack((dot(self.Cz, \
                    dot(matpower(self.A, i), self.Bv )), Nztmp[:-self.nyBv]))))
            else:
                if dot(self.Cz, dot(matpower(self.A, i), self.Bv )).ndim == 1:
                    Nztmp = np.vstack((Nztmp, np.hstack((dot(self.Cz, \
                        dot(matpower(self.A, i), self.Bv )), Nztmp[-self.nxCz:,:-self.nyBv][0]))))
                else:
                    Nztmp = np.vstack((Nztmp, np.hstack((dot(self.Cz, \
                        dot(matpower(self.A, i), self.Bv )), Nztmp[-self.nxCz:,:-self.nyBv]))))
        self.Nz = Nztmp

    def mat_sizes(self):
        if self.A.ndim == 1:
            self.nxA = 1
            self.nyA = 0
        elif self.A.ndim == 2:
            self.nxA = np.shape(self.A)[0]
            self.nyA = np.shape(self.A)[1]
        else:
            raise Exception("The 'A' matrix has > 2 dimensions")

        if self.Bu.ndim == 1:
            self.nxBu = 1
            self.nyBu = 0
        elif self.Bu.ndim == 2:
            self.nxBu = np.shape(self.Bu)[0]
            self.nyBu = np.shape(self.Bu)[1]
        else:
            raise Exception("The 'Bu' matrix has > 2 dimensions")
        
        if self.Bv.ndim == 1:
            self.nxBv = 1
            self.nyBv = 0
        elif self.Bv.ndim == 2:
            self.nxBv = np.shape(self.Bv)[0]
            self.nyBv = np.shape(self.Bv)[1]
        else:
            raise Exception("The 'Bv' matrix has > 2 dimensions")

        if self.Bd.ndim == 1:
            self.nxBd = 1
            self.nyBd = 0
        elif self.Bd.ndim == 2:
            self.nxBd = np.shape(self.Bd)[0]
            self.nyBd = np.shape(self.Bd)[1]
        else:
            raise Exception("The 'Bd' matrix has > 2 dimensions")

        if self.Cy.ndim == 1:
            self.nxCy = 1
            self.nyCy = 0
        elif self.Cy.ndim == 2:
            self.nxCy = np.shape(self.Cy)[0]
            self.nyCy = np.shape(self.Cy)[1]
        else:
            raise Exception("The 'Cy' matrix has > 2 dimensions")

        if self.Cz.ndim == 1:
            self.nxCz = 1
            self.nyCz = 0
        elif self.Cz.ndim == 2:
            self.nxCz = np.shape(self.Cz)[0]
            self.nyCz = np.shape(self.Cz)[1]
        else:
            raise Exception("The 'Cz' matrix has > 2 dimensions")

        if self.Dy.ndim == 1:
            self.nxDy = 1
            self.nyDy = 0
        elif self.Dy.ndim == 2:
            self.nxDy = np.shape(self.Dy)[0]
            self.nyDy = np.shape(self.Dy)[1]
        else:
            raise Exception("The 'Dy' matrix has > 2 dimensions")

        if self.Dz.ndim == 1:
            self.nxDz = 1
            self.nyDz = 0
        elif self.Dz.ndim == 2:
            self.nxDz = np.shape(self.Dz)[0]
            self.nyDz = np.shape(self.Dz)[1]
        else:
            raise Exception("The 'Dz' matrix has > 2 dimensions")