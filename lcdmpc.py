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
from pyoptsparse import Optimization, SNOPT

class LCDMPC():
    def __init__(self):
        self.subsystems = []

    def build_subsystem(self, A, Bu, Bv, Bd, Cy, Cz, Dyu, Dyv, Dzu, Dzv, 
            inputs, outputs, refs, horiz_len, Beta, cons, nodeID=None, nodeName=None):
        # create subsystem object
        subsys = subsystem(self, A, Bu, Bv, Bd, Cy, Cz, Dyu, Dyv, Dzu, Dzv, 
            inputs, outputs, refs, horiz_len, Beta, cons, nodeID=nodeID, 
            nodeName=nodeName)
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

    def update_downstream_outputs(self):
        """
        Update the z vectors of the subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_Z()

    def communicate(self):
        """
        Exchange information between subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_V(self)
            subsys.update_Psi(self)

    def optimize_all(self):
        """
        Loop through all subsystems and perform optimization.
        """
        for subsys in self.subsystems:
            subsys.uOpt = subsys.optimize()

    def convex_sum_cont(self):
        """
        Perform the convex combination of the new control action and 
        the previous control action.
        """
        for subsys in self.subsystems:
            subsys.uConv = subsys.Beta*subsys.uConv + \
                           (1 - subsys.Beta)*subsys.uOpt

    def calculate_sensitivities(self):
        """
        Calculate the sensitivities of the subsystem to the upstream 
        systems.
        """
        for subsys in self.subsystems:
            subsys.calc_sens()

    def update_states(self):
        """
        Update the states of the subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_x()

    def update_subsystem_outputs(self):
        """
        Update the subsystem outpus, 'y'.
        """
        for subsys in self.subsystems:
            subsys.update_y()
        

class subsystem():
    def __init__(self, obj, A, Bu, Bv, Bd, Cy, Cz, Dyu, Dyv, Dzu, Dzv, 
                 inputs, outputs, refs, horiz_len, Beta, cons, upstream=None, 
                 downstream=None, nodeID=None, nodeName=None, optOptions=None):
        self.A = A
        self.Bu = Bu
        self.Bv = Bv
        self.Bd = Bd
        self.Cy = Cy
        self.Cz = Cz
        self.Dyu = Dyu
        self.Dyv = Dyv
        self.Dzu = Dzu
        self.Dzv = Dzv
        # self.Q = Q
        # self.S = S
        self.inputs = inputs
        self.outputs = outputs
        self.refs = refs
        self.horiz_len = horiz_len
        self.Beta = Beta
        self.cons = cons
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
        if optOptions is not None:
            self.optOptions = optOptions
        else:
            self.optOptions = {'Major feasibility tolerance' : 1e-1}

        self.mat_sizes()
        self.sys_matrices()
        self.x0 = np.zeros(self.nxA)
        self.x1 = []
        self.u0 = np.zeros(self.nxA*self.horiz_len)
        self.uConv = self.u0
        self.u0_min = self.cons['lower']*self.horiz_len
        self.u0_max = self.cons['upper']*self.horiz_len
        self.uOpt = []
        # self.V = np.zeros((self.nyNy, self.nxCz))
        self.V = np.zeros(self.nyNy)
        self.y = []
        # self.Z = np.zeros((self.nxNz, np.shape(self.V)[1]))
        self.Z = np.zeros(self.nxNz)
        # self.Gamma = np.zeros((self.nxNz, self.nxCz))
        self.Gamma = np.zeros(self.nxNz)
        # self.Psi = np.zeros((self.nxMz, self.nxCz))
        self.Psi = np.zeros(self.nxMz)
        self.refs = self.refs*self.horiz_len
        self.sol = []

        self.E = dot(dot(tp(self.Ny), self.Q), self.Ny)

    def calculate_horizon(self):
        pass

    def set_opt_bounds(self, low, high):
        pass

    # def optimize(self):
    #     P_qp = self.H # quadratic term
    #     q_qp = self.F # linear term
    #     G_qp = TODO
    #     h_qp = np.hstack(-1*self.lb, self.ub)
    #     self.u = cvxopt_solve_qp(P_qp, q_qp)

    # def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None):    
    #     P = .5 * (P + P.T)  # make sure P is symmetric
    #     args = [cvx.matrix(P), cvx.matrix(q)]
    #     if G is not None:
    #         args.extend([cvx.matrix(G), cvx.matrix(h)])
    #         if A is not None:
    #             args.extend([cvx.matrix(A), cvx.matrix(b)])
    #     sol = cvxopt.solvers.qp(*args)
    #     if 'optimal' not in sol['status']:
    #         return None
    #     return np.array(sol['x']).reshape((P.shape[1],))

    # def obj_func(self, control):
    #     # U^T*H*U + 2*U^T*F + V^T*E*V + 2*V^T*T
    #     self.update()
    #     self.U = control
    #     return dot(dot(tp(self.U), self.H), self.U) + 2*dot(tp(self.U), F) \
    #         + dot(dot(tp(self.V), self.E), self.V) + 2*dot(tp(self.V), self.T)

    def optimize(self):
        self.update()

        optProb = Optimization('LCDMPC', self.obj_func)
        optProb.addVarGroup('U', len(self.u0), type='c', lower=self.u0_min, 
                            upper=self.u0_max, value=self.u0)
        optProb.addObj('obj')

        # optProb = self.add_constraints(optProb)

        opt = SNOPT(optOptions=self.optOptions)
        sol = opt(optProb, sens=self.sens)

        self.sol = sol

        print(sol)

        return list(sol.getDVs().values())[0]

    def obj_func(self, varDict):
        # U^T*H*U + 2*U^T*F + V^T*E*V + 2*V^T*T
        self.uOpt = varDict['U']
        funcs = {}

        funcs['obj'] = dot(dot(tp(self.uOpt), self.H), self.uOpt) \
                     + 2*dot(tp(self.uOpt), self.F) \
                     + dot(dot(tp(self.V), self.E), self.V) \
                     + 2*dot(tp(self.V), self.T)

        print('obj value: ', funcs['obj'])

        fail = False
        return funcs, fail

    def sens(self, varDict, funcs):
        self.uOpt = varDict['U']
        funcsSens = {}

        funcsSens['obj', 'U'] = 2*dot(self.H, self.uOpt) + 2*self.F

        fail = False
        return funcsSens, fail

    def add_constraints(self, optProb):
        for con in self.cons:
            optProb.addConGroup(con['name'], con['ncon'], con['lower'], 
                        con['upper'], con['wrt'], con['linear'], con['jac'])

        return optProb

    def update(self):
        self.H = dot(dot(tp(self.My), self.Q), self.My) + self.S
        self.E_1 = self.E
        self.E = dot(dot(tp(self.Ny), self.Q), self.Ny)
        self.F = dot(dot(tp(self.My), self.Q), (dot(self.Fy, self.x0) \
               + dot(self.Ny, self.V) - self.refs)) \
               + 0.5*dot(tp(self.Mz), self.Psi)
        self.T = dot(dot(tp(self.Ny), self.Q), (dot(self.Fy, self.x0) \
               - self.refs)) + 0.5*dot(tp(self.Nz), self.Psi)

    def calc_sens(self):
        self.gamma = 2*(dot(self.E_1, self.V) + self.T \
                   + dot(dot(dot(tp(self.Ny), self.Q), self.My), self.uConv))

    def update_subsys(self):
        self.update_x()
        self.update_y()
        self.update_Z()

    def update_V(self, obj):
        # TODO: figure out how to make this work when z is more than one value
        for upstream in self.upstream:
            self.V = obj.subsystems[upstream].Z

    def update_Psi(self, obj):
        # TODO: figure out how to make this work when z is more than one value
        for upstream in self.upstream:
            self.Psi = obj.subsystems[upstream].Gamma

    def update_x(self):
        # TODO: add in self.d to class
        self.x1 = dot(self.A, self.x0) + dot(self.Bu, self.uConv[0:self.nxBu]) \
            + dot(self.Bv, self.V[0:self.nxBv])# + dot(self.Bd, self.d)
        self.x0 = self.x1

    def update_y(self):
        self.y = dot(self.Cy, self.x0) + dot(self.Dyu, self.uConv[0:self.nxDyu]) \
            + dot(self.Dyv, self.V[0:self.nxDyv])

    def update_Z(self):
        self.Z = dot(self.Fz, self.x0) + dot(self.Mz, self.uConv) \
            + dot(self.Nz, self.V)

    def sys_matrices(self):       
        self.Fy = np.array([dot(self.Cy, matpower(self.A, i)) \
                  for i in range(1, self.horiz_len + 1)])
        self.Fy = np.reshape(self.Fy, (self.nxCy*self.horiz_len, self.nxA), order='C')
        # if self.Fy[0].ndim > 1:
        #     self.Fy = np.concatenate(self.Fy, axis=0)
        # else:
        #     self.Fy = np.concatenate(self.Fy)

        self.Fz = np.array([dot(self.Cz, matpower(self.A, i)) \
                  for i in range(0, self.horiz_len)])
        self.Fz = np.reshape(self.Fz, (self.nxCz*self.horiz_len, self.nxA))
        # if self.Fz[0].ndim > 1:
        #     self.Fz = np.concatenate(self.Fz, axis=1)
        # else:
        #     self.Fz = np.concatenate(self.Fz)

        # Mytmp = dot(self.Cy, self.Bu)
        # MytmpShape = np.shape(Mytmp)
        # for i in range(self.horiz_len - 1):
        #     Mytmp = np.hstack((Mytmp, np.zeros(MytmpShape)))
        # for i in range(1, self.horiz_len):
        #     if Mytmp.ndim == 1:
        #         Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
        #             dot(matpower(self.A, i), self.Bu )), Mytmp[:-self.nyBu]))))
        #     else:
        #         if dot(self.Cy, dot(matpower(self.A, i), self.Bu )).ndim == 1:
        #             Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
        #                 dot(matpower(self.A, i), self.Bu )), Mytmp[-self.nxCy:,:-self.nyBu][0]))))
        #         else:
        #             Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
        #                 dot(matpower(self.A, i), self.Bu )), Mytmp[-self.nxCy:,:-self.nyBu]))))
        # self.My = Mytmp

        Mytmp = dot(self.Cy, self.Bu)
        MytmpShape = np.shape(Mytmp)
        Mytmp = np.hstack((Mytmp, self.Dyu))
        for i in range(self.horiz_len - 2):
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

        Mztmp0 = self.Dzu
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

        # TODO: need to update with Dyv
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

        if self.Fy.ndim == 1:
            self.nxFy = 1
            self.nyFy = 0
        else:
            self.nxFy = np.shape(self.Fy)[0]
            self.nyFy = np.shape(self.Fy)[1]

        if self.Fz.ndim == 1:
            self.nxFz = 1
            self.nyFz = 0
        else:
            self.nxFz = np.shape(self.Fz)[0]
            self.nyFz = np.shape(self.Fz)[1]

        if self.My.ndim == 1:
            self.nxMy = 1
            self.nyMy = 0
        else:
            self.nxMy = np.shape(self.My)[0]
            self.nyMy = np.shape(self.My)[1]

        if self.Mz.ndim == 1:
            self.nxMz = 1
            self.nyMz = 0
        else:
            self.nxMz = np.shape(self.Mz)[0]
            self.nyMz = np.shape(self.Mz)[1]

        if self.Ny.ndim == 1:
            self.nxNy = 1
            self.nyNy = 0
        else:
            self.nxNy = np.shape(self.Ny)[0]
            self.nyNy = np.shape(self.Ny)[1]

        if self.Nz.ndim == 1:
            self.nxNz = 1
            self.nyNz = 0
        else:
            self.nxNz = np.shape(self.Nz)[0]
            self.nyNz = np.shape(self.Nz)[1]

        # TODO: make this work for user-supplied Q's and S's
        self.Q = np.diag(np.ones(self.nxMy))
        self.S = np.diag(np.ones(self.nyMy))

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

        if self.Dyu.ndim == 1:
            self.nxDyu = 1
            self.nyDyu = 0
        elif self.Dyu.ndim == 2:
            self.nxDyu = np.shape(self.Dyu)[0]
            self.nyDyu = np.shape(self.Dyu)[1]
        else:
            raise Exception("The 'Dy' matrix has > 2 dimensions")

        if self.Dyv.ndim == 1:
            self.nxDyv = 1
            self.nyDyv = 0
        elif self.Dyv.ndim == 2:
            self.nxDyv = np.shape(self.Dyv)[0]
            self.nyDyv = np.shape(self.Dyv)[1]
        else:
            raise Exception("The 'Dy' matrix has > 2 dimensions")

        # if self.Dz.ndim == 1:
        #     self.nxDz = 1
        #     self.nyDz = 0
        # elif self.Dz.ndim == 2:
        #     self.nxDz = np.shape(self.Dz)[0]
        #     self.nyDz = np.shape(self.Dz)[1]
        # else:
        #     raise Exception("The 'Dz' matrix has > 2 dimensions")