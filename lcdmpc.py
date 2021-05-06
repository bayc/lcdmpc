# Copyright 2020 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not 
# use this file except in compliance with the License. You may obtain a copy of 
# the License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT 
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the 
# license for the specific language governing permissions and limitations under 
# the License.

import numpy as np
import pandas as pd
import copy
from numpy import dot as dot
from numpy import transpose as tp
from numpy.linalg import matrix_power as matpower
from pyoptsparse import Optimization, SNOPT

class LCDMPC():
    def __init__(self, start_time, time_step):
        self.subsystems = []
        self.current_time = start_time
        self.time_step = time_step

    def build_subsystem(self, idn, control_model, truth_model, inputs, outputs,
        horiz_len, Beta, disturbance_file, refs=None, refs_total=None, nodeID=None, nodeName=None,
        optOptions=None):
        # create subsystem object
        subsys = subsystem(self, idn, control_model, truth_model, inputs,
            outputs, horiz_len, Beta, disturbance_file, self.current_time,
            refs=refs, refs_total=refs_total, nodeID=nodeID, nodeName=nodeName,
            optOptions=optOptions)
        # append it to subsystem list
        self.subsystems.append(subsys)

    def simulate_truth_model(self):
        # apply control action and simulate truth models
        outputs = []
        for subsys in self.subsystems:
            u_applied = subsys.uConv[:subsys.control_model.numDVs]
            # print('u_applied: ', u_applied)
            disturb = subsys.d
            outputs.append(subsys.simulate_truth(self.current_time, u_applied, disturb))
            # print('outputs: ', outputs)

        # TODO: change to date-time; update in other places
        self.current_time = self.current_time + 1

        return outputs

    def update_control_filter(self):
        for subsys in self.subsystems:
            subsys.update_control_filter()

    def update_forecast_inputs(self):
        # update the forecasts for each of the subsystem models

        for subsys in self.subsystems:
            subsys.update_forecast(self.current_time)

    def optimize(self):
        pass

    def reinitialize(self):
        pass

    def build_interconnections(self, interconnections):
        """
        Accept inteconnections from subsystems and setup 
        interconnection. matrix
        
        Args:
            interconnections (list): A list of tuples containing the 
                upstream and downstream nodeIds.
        """
        if interconnections==None:
            for subsys in self.subsystems:
                subsys.downstream.append('None')
                subsys.upstream.append('None')
                subsys.control_model.num_upstream = 0
                subsys.control_model.num_downstream = 0
        else:
            for pair in interconnections:
                up = pair[0]
                down = pair[1]

                self.subsystems[up].downstream.append(down)
                self.subsystems[down].upstream.append(up)
            
            for subsys in self.subsystems:
                subsys.control_model.num_upstream = len(subsys.upstream)
                subsys.control_model.num_downstream = len(subsys.downstream)

    def update_downstream_outputs(self):
        """
        Update the z vectors of the subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_Z()
            # print('self.Z subsys: ', subsys.Z)

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
            subsys.uConv = subsys.Beta*np.array(subsys.uConv) + \
                           (1 - subsys.Beta)*np.array(subsys.uOpt)
            subsys.process_uConv()

    def calculate_sensitivities(self):
        """
        Calculate the sensitivities of the subsystem to the upstream 
        systems.
        """
        gamma = []
        for i, subsys in enumerate(self.subsystems):
            gamma.append(subsys.calc_sens())

        return gamma

    def calc_obj(self):
        for subsys in self.subsystems:
            subsys.calc_obj()

    def update_states(self):
        """
        Update the states of the subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_x()

    def update_subsystem_outputs(self):
        """
        Update the subsystem outputs, 'y'.
        """
        for subsys in self.subsystems:
            subsys.update_y()

    def update_inputs_for_linearization(self):
        """
        Updates the inputs for the subsystems to correctly relinearize, if 
        needed.
        """
        for subsys in self.subsystems:
            subsys.update_inputs()
            # subsys.update_disturbances()

    def relinearize_subsystem_models(self):
        """
        Uses the latest input data to relinearize the subsystem models.
        """
        for subsys in self.subsystems:
            subsys.relinearize(subsys.control_model, subsys.inputs, 
                               subsys.d, subsys.outputs)


class subsystem():
    def __init__(self, obj, idn, control_model, truth_model, inputs, outputs,
                 horiz_len, Beta, disturbance_file, current_time, 
                 refs=None, refs_total=None, upstream=None, downstream=None, 
                 nodeID=None, nodeName=None, optOptions=None):
        self.gamma_scale = control_model.gamma_scale

        # self.Q = Q
        # self.S = S
        # self.count = 0
        self.idn = idn
        self.current_time = current_time
        self.refs_plot = []
        self.outputs = []
        
        self.truth_model = truth_model
        self.control_model = control_model

        self.horiz_len = horiz_len
        self.Beta = Beta
        
        if refs is not None:
            self.refs = refs 
            self.refs_const = copy.deepcopy(self.refs)
            self.refs = self.control_model.process_refs(self.refs)
            self.refs = self.refs*self.horiz_len
            self.refs_const = self.refs_const*self.horiz_len

        if refs_total is not None:
            self.refs_total = refs_total
            self.refs = self.control_model.process_refs_horiz(refs_total=self.refs_total, current_time=self.current_time)
        # self.refs = self.refs - self.control_model.truth_model_Pwr \
        #     + self.control_model.Cy_lin \
        #     + self.control_model.Dyu_lin \
        #     + self.control_model.Dyd_lin
        # print('initial refs: ', self.refs)

        self.disturbance_file = disturbance_file
        self.disturbance_data = pd.read_csv(self.disturbance_file)
        self.update_forecast(current_time)

        self.y = self.control_model.Cy_mean_outputs

        self.V = np.zeros((np.shape(self.control_model.Bv)[1]*self.horiz_len, 1))

        self.relinearize(control_model, inputs, self.d, outputs)

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
            self.optOptions = {'Major feasibility tolerance' : 1e-4}
        print(self.optOptions)

        self.x0 = np.zeros((self.nxA, 1))
        # self.x0 = np.array([[0.0] for _ in range(self.nxA)])
        self.x1 = []
        self.u0 = np.zeros((self.nyBu*self.horiz_len, 1))
        self.uConv = self.u0
        self.u0_min = 0.0
        self.u0_max = 100.0
        # self.uOpt = []
        self.uOpt = [-100, 0.8, 12.8]*self.horiz_len
        self.uOpt = np.array(self.uOpt).reshape(len(self.uOpt), 1)
        
        self.D = np.zeros((self.nyPy, 1))
        # self.V = np.array([[0.0] for _ in range(self.nyNy)])
        
        self.Z = np.zeros((self.nxNz, 1))
        self.gamma = np.zeros((self.nxNz, 1))
        self.Psi = np.zeros((self.nxMz, 1))
        # print('initial horizon refs: ', self.refs)
        self.sol = []

    def update_control_filter(self):
        # print('self.y before filter: ', self.y)
        self.x0 = self.control_model.filter_update(self.x0, self.truth_model.outputs)
        # print('self.x0 after filter update: ', self.x0)

    def simulate_truth(self, current_time, inputs, disturb):
        self.current_time = current_time
        # print(self.truth_model)
        outputs = self.truth_model.simulate(
            current_time,
            inputs,
            disturb,
            self.V[:self.control_model.num_downstream]
        )
        # print('outputs: ', outputs)
        self.outputs.append(outputs)

        return outputs

    def update_forecast(self, current_time):
        # disturbance horizon
        self.d = self.truth_model.get_forecast(current_time, self.disturbance_data)
        # print('self.d: ', self.d)
        self.D = self.control_model.get_forecast(current_time, self.disturbance_data)
        # print('self.D: ', self.D)

    def relinearize(self, control_model, inputs, disturb, outputs):

        # self.refs[1::2] = [self.refs_const[1] - control_model.truth_model_Pwr]*self.horiz_len
        # print('power refs: ', self.refs[1::2])
        # import matplotlib.pyplot as plt
        # plt.figure()
        # plt.plot(self.refs)
        # plt.show()
        # lkj
        # print('self.refs before: ', self.refs)
        # print('dy: ', (self.y[0] - self.control_model.Cy_mean_outputs))
        # self.refs = [val - (self.y[0] - self.control_model.Cy_mean_outputs) for val in self.refs_const]
        # print('self.refs after: ', self.refs)

        self.inputs = inputs
        self.outputs = outputs

        self.control_model = control_model

        control_model.reinit(inputs, disturb)

        # if self.current_time == 165:
        #     self.refs_const[1::2] = [self.refs_const[1] + 20]*self.horiz_len
        # if self.current_time == 180:
        #     self.refs_const[1::2] = [self.refs_const[1] - 10]*self.horiz_len

        if hasattr(self, 'refs_total'):
            self.refs = self.control_model.process_refs_horiz(
                refs_total=self.refs_total, current_time=self.current_time
            )
        else:
            self.refs = self.control_model.process_refs_horiz(
                self.refs, self.refs_const
            )
        
        # self.refs = np.array(self.refs_const) \
        #     - np.array([0, self.control_model.truth_model_Pwr]*self.horiz_len) \
        #     + np.array([self.control_model.Cy_lin]*self.horiz_len).flatten() \
        #     + np.array([self.control_model.Dyu_lin]*self.horiz_len).flatten() \
        #     + np.array([self.control_model.Dyd_lin]*self.horiz_len).flatten()
        # print('**********Refs: ', self.refs)
        # self.refs_plot.append(self.refs_const[0:2])

        self.A = control_model.A
        self.Bu = control_model.Bu
        self.Bv = control_model.Bv
        self.Bd = control_model.Bd
        self.Cy = control_model.Cy
        self.Cz = control_model.Cz
        self.Dyu = control_model.Dyu
        # if self.idn == 1:
        #     print('##############: ', self.Dyu)
        self.Dyv = control_model.Dyv
        self.Dyd = control_model.Dyd
        self.Dzu = control_model.Dzu
        self.Dzv = control_model.Dzv
        self.Dzd = control_model.Dzd

        self.mat_sizes()
        self.sys_matrices()

        # print('size My: ', self.nxMy, self.nyMy)
        # print('size Ny: ', self.nxNy, self.nyNy)
        # print('size Py: ', self.nxPy, self.nyPy)
        # lkj

        self.E = dot(dot(tp(self.Ny), self.Q), self.Ny)

        self.d = disturb

    def calculate_horizon(self):
        # for building model, assume room_temp measurement
        # to be last measurement over horizon
        self.update_Y()

    def set_opt_bounds(self, low, high):
        pass

    def optimize(self):
        self.update()

        optProb = Optimization('LCDMPC', self.obj_func)
        # optProb.addVarGroup('U', len(self.u0), type='c', lower=self.u0_min, 
        #                     upper=self.u0_max, value=self.u0)

        optProb = self.control_model.add_var_group(optProb)

        optProb = self.control_model.add_con_group(optProb)

        optProb.addObj('obj')

        # optProb = self.add_constraints(optProb)

        opt = SNOPT(optOptions=self.optOptions)
        opt.setOption('Print file', value=self.optOptions['Print file'])
        opt.setOption('Summary file', value=self.optOptions['Summary file'])
        opt.setOption('Iterations limit', 100)
        # sol = opt(optProb, sens=self.sens)
        sol = opt(optProb, sens='FDR')
        # sol = opt(optProb)

        self.sol = sol

        # print(sol)

        self.uOpt = self.control_model.parse_sol_vars(sol)
        return self.uOpt

    def obj_func(self, varDict):
        # Parse the optimization variables defined in the control model file
        self.uOpt = self.control_model.parse_opt_vars(varDict)
        
        # Setup the objective function for LC-DMPC
        funcs = {}
        # print('uOpt shape: ', np.shape(self.uOpt))
        # print('self.H shape: ', np.shape(self.H))
        # print('self.F shape: ', np.shape(self.F))
        # print('self.V shape: ', np.shape(self.V))
        # print('self.E shape: ', np.shape(self.E))
        # print('self.T shape: ', np.shape(self.T))
        # print('uopt: ', self.uOpt)
        # print('idn: ', self.idn)
        # print('obj1: ', dot(dot(tp(self.uOpt), self.H), self.uOpt))
        # print('obj2: ', 2*dot(tp(self.uOpt), self.F))
        funcs['obj'] = (dot(dot(tp(self.uOpt), self.H), self.uOpt) \
                     + 2*dot(tp(self.uOpt), self.F) \
                     + dot(dot(tp(self.V), self.E), self.V) \
                     + 2*dot(tp(self.V), self.T))
        # print('obj_func: ', np.shape(funcs['obj']))
        # Compute constraints, if any are defined for control model
        # print('!!!!!: ', funcs['obj'])
        self.update_Y()
        # print('self.Y: ', self.Y)
        funcs = self.control_model.compute_cons(funcs, self.uOpt, self.Y)

        fail = False
        return funcs, fail

    def calc_obj(self):

        obj_func = (dot(dot(tp(self.uOpt), self.H), self.uOpt) \
                     + 2*dot(tp(self.uOpt), self.F) \
                     + dot(dot(tp(self.V), self.E), self.V) \
                     + 2*dot(tp(self.V), self.T))
        # print('opt_control: ', self.uConv[:self.control_model.numDVs])
        # print('objective function: ', obj_func)
        # print('self.Y: ', self.Y)
        # print('self.Nz: ', self.Nz)
        # print('self.Dy: ', self.control_model.Cy)
        # print('self.My:', self.Fy)
        
        # print('optimized variables: ', self.uOpt)
        # print('convex combination: ', self.uConv)
        # print('UPDATEY SELF.D: ', self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ))
        # print('UPDATEY uconv: ', self.V)
        # print('self.Psi: ', self.Psi)
        # print('self.V: ', self.V)

    def sens(self, varDict, funcs):
        # Parse the optimization variables defined in the control model file
        self.uOpt = self.control_model.parse_opt_vars(varDict)

        funcsSens = self.control_model.sensitivity_func()

        funcsSens = {}
        # funcsSens = {
        #     ('obj', 'yaw_r') : (2*dot(self.H, self.uOpt) + 2*self.F)[0::2],
        #     ('obj', 'Ct_r') : (2*dot(self.H, self.uOpt) + 2*self.F)[1::2]
        # }
        # a = np.array(self.uOpt[1::3]) + self.truth_model.T_z
        # b = self.truth_model.T_z
        # b = (a.diagonal() + self.truth_model.T_z)
        # print('a: ', self.uOpt[2::3] - self.truth_model.T_z.flatten())
        # print('b: ', -1*np.diag(self.uOpt[2::3] - self.truth_model.T_z.flatten()))
        # print(np.diag(b))
        # print('shape: ', np.diag(np.ones(self.horiz_len)))
        # print(type(self.truth_model.T_z))
        # print('stuff: ', np.fill_diagonal(a, a.diagonal() + self.truth_model.T_z))
        # np.fill_diagonal(a, a.diagonal() / c)
        funcsSens = {
            ('obj', 'Qhvac') : (2*dot(self.H, self.uOpt) + 2*self.F)[0::3],
            ('obj', 'ms_dot') : (2*dot(self.H, self.uOpt) + 2*self.F)[1::3],
            ('obj', 'T_sa') : (2*dot(self.H, self.uOpt) + 2*self.F)[2::3],
            ('hvac_con', 'Qhvac') : np.diag(np.ones(self.horiz_len)),
            ('hvac_con', 'ms_dot') : -1*np.diag(self.uOpt[2::3] \
                - self.truth_model.T_z.flatten()),
            ('hvac_con', 'T_sa') : np.diag(-1*np.array(self.uOpt[1::3])),
            ('T_building_con', 'Qhvac') : self.My[0::2, 0::3],
            ('T_building_con', 'ms_dot') : self.My[0::2, 1::3],
            ('T_building_con', 'T_sa') : self.My[0::2, 2::3]}
        #     #  ('con1', 'Qhvac') : np.ones(5)*1.0,
        #     #  ('con1', 'ms_dot') : -1*np.ones(5)*self.T_sa[0],
        #     #  ('con1', 'T_sa') : -1*np.ones(5)*self.ms_dot[0]}
        #     ('con1', 'Qhvac') : np.eye(5)*1.0,
        #     ('con1', 'ms_dot') : -1*np.eye(5)*self.T_sa,
        #     ('con1', 'T_sa') : -1*np.eye(5)*self.ms_dot,
        #     ('con6', 'Qhvac') : np.eye(5)*np.diag(self.model.A),
        #     ('con6', 'ms_dot') : np.eye(5)*0.0,
        #     ('con6', 'T_sa') : np.eye(5)*1.0}
        #     ('con6', 'ms_dot') : np.diag(
        #         (3*self.model.a0*self.ms_dot**2 \
        #         + 2*self.model.a1*self.ms_dot \
        #         + self.model.a0 + 1.005/self.model.hvac_cop \
        #         * (0.3*self.model.T_oa + (1 - 0.3)*self.Y - self.T_sa))._value
        #     ),
        #     ('con6', 'T_sa'): np.diag(
        #         -1*1.005/self.model.hvac_cop*self.ms_dot
        #     )}
        #     ('con2', 'Qhvac') : 1.0,
        #     ('con2', 'ms_dot') : -1*self.T_sa[1],
        #     ('con2', 'T_sa') : -1*self.ms_dot[1],
        #     ('con3', 'Qhvac') : 1.0,
        #     ('con3', 'ms_dot') : -1*self.T_sa[2],
        #     ('con3', 'T_sa') : -1*self.ms_dot[2],
        #     ('con4', 'Qhvac') : 1.0,
        #     ('con4', 'ms_dot') : -1*self.T_sa[3],
        #     ('con4', 'T_sa') : -1*self.ms_dot[3],
        #     ('con5', 'Qhvac') : 1.0,
        #     ('con5', 'ms_dot') : -1*self.T_sa[4],
        #     ('con5', 'T_sa') : -1*self.ms_dot[4]}

        fail = False
        return funcsSens, fail

    # def add_constraints(self, optProb):
    #     for con in self.cons:
    #         optProb.addConGroup(con['name'], con['ncon'], con['lower'], 
    #                     con['upper'], con['wrt'], con['linear'], con['jac'])

    #     return optProb

    def update(self):
        self.H = dot(dot(tp(self.My), self.Q), self.My) + self.S
        # self.E_1 = self.E
        self.E = dot(dot(tp(self.Ny), self.Q), self.Ny)
        # print('My: ', np.shape(self.My))
        # print('Q: ', np.shape(self.Q))
        # print('Fy: ', np.shape(self.Fy))
        # print('x0: ', np.shape(self.x0))
        # print('Ny: ', np.shape(self.Ny))
        # print('V: ', np.shape(self.V))
        # print('Py: ', np.shape(self.Py))
        # print('D: ', np.shape(self.D))
        # print('Bd_mean_inputs: ', np.shape(self.control_model.Bd_mean_inputs))
        # print('refs: ', np.shape(self.refs))
        # print('special: ', np.shape(self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         )))
        # print('tile: ', np.shape(np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         )))
        # print('Mz: ', np.shape(self.Mz))
        # print('Psi: ', np.shape(self.Psi))
        # print('1: ', np.shape(dot(tp(self.My), self.Q)))
        # print('2: ', np.shape(dot(self.Fy, self.x0)))
        # print('3: ', np.shape(dot(self.Ny, self.V)))
        # print('4: ', np.shape(dot(self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ).reshape(np.shape(self.Py)[1], 1)) - self.refs))
        # print('5: ', np.shape(dot(tp(self.Mz), self.Psi)))
        # print('6: ', np.shape(dot(dot(tp(self.My), self.Q), (dot(self.Fy, self.x0) \
        #        + dot(self.Ny, self.V) + dot(self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ).reshape(np.shape(self.Py)[1], 1)) - self.refs))))
        # self.F = dot(dot(tp(self.My), self.Q), (dot(self.Fy, self.x0) \
        #        + dot(self.Ny, self.V) + dot(self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ).reshape(np.shape(self.Py)[1], 1)) - self.refs)) \
        #        + 0.5*dot(tp(self.Mz), self.Psi)
        # print('shape Cy: ', np.shape(self.Cy))
        # print('Cy: ', self.Cy)
        # print('shape Bv: ', np.shape(self.Bv))
        # print('shape My: ', np.shape(self.My))
        # print('shape Q: ', np.shape(self.Q))
        # print('shape Fy: ', np.shape(self.Fy))
        # print('shape x0: ', np.shape(self.x0))
        # print('shape Ny: ', np.shape(self.Ny))
        # print('shape V: ', np.shape(self.V))
        # print('shape Py: ', np.shape(self.Py))
        # print('shape D: ', self.D)
        # print('shape tile: ', np.tile(
        #             self.control_model.Bd_mean_inputs, (self.horiz_len, 1)
        #         ))
        # print('shape refs: ', np.shape(self.refs))
        # print('Y: ', dot(self.Fy, self.x0) \
        #        + dot(self.Ny, self.V) + dot(self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, (self.horiz_len, 1)
        #         )))

        # print('term1: ', dot(dot(tp(self.My), self.Q), (dot(self.Fy, self.x0) \
        #        + dot(self.Ny, self.V) + dot(self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, (self.horiz_len, 1)
        #         )) - self.refs)))
        # print('term2: ', 0.5*dot(tp(self.Mz), self.Psi))

        # print('term3: ', dot(dot(tp(self.Ny), self.Q), (dot(self.Fy, self.x0) \
        #        + dot(self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ).reshape(np.shape(self.Py)[1], 1)) - self.refs)))
        # print('term3a: ', dot(tp(self.Ny), self.Q))
        # print('term3b: ', dot(self.Fy, self.x0))
        # print('term3c: ', dot(self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, (self.horiz_len, 1)
        #         )))
        # print('term3d: ', self.refs)
        # print('shape of Mz: ', np.shape(self.Mz))
        # print('shape of Psi: ', np.shape(self.Psi))
        # print('term4: ', 0.5*dot(tp(self.Nz), self.Psi))
        # print('self.idn: ', self.idn)


        self.F = dot(dot(tp(self.My), self.Q), (dot(self.Fy, self.x0) \
               + dot(self.Ny, self.V) + dot(self.Py, self.D - np.tile(
                    self.control_model.Bd_mean_inputs, (self.horiz_len, 1)
                )) - self.refs)) \
               + 0.5*dot(tp(self.Mz), self.Psi)
        self.T = dot(dot(tp(self.Ny), self.Q), (dot(self.Fy, self.x0) \
               + dot(self.Py, self.D - np.tile(
                    self.control_model.Bd_mean_inputs, self.horiz_len
                ).reshape(np.shape(self.Py)[1], 1)) - self.refs)) + 0.5*dot(tp(self.Nz), self.Psi)

    def calc_sens(self):
        # self.gamma = self.control_model.calculate_sens(self)
        # self.gamma = 2*(dot(self.E_1, self.V) + self.T \
                #    + dot(dot(dot(tp(self.Ny), self.Q), self.My), self.uConv))
        self.gamma = 2*self.gamma_scale*(dot(self.E, self.V) + self.T \
                   + dot(dot(dot(tp(self.Ny), self.Q), self.My), self.uConv))
        # if self.idn ==0:
        #     print('self.E: ', self.E)
        #     print('self.V: ', self.V)
        #     print('self.T: ', self.T)
        #     print('self.Ny: ', self.Ny)
        #     print('self.Q: ', self.Q)
        #     print('self.My: ', self.My)
        #     print('self.uConv: ', self.uConv)
        # if self.idn == 0:
        print('subsys idn: ', self.idn)
        print(' self.Z: ', self.Z)
        print(' self.V: ', self.V)
        print(' gamma: ', self.gamma)

        return self.gamma

    def update_inputs(self):
        self.inputs = self.control_model.update_inputs(
            self.x0,
            self.uConv[:self.control_model.numDVs],
            self.truth_model.outputs
        )

    def update_disturbances(self):
        self.disturb = self.control_model.update_disturbances(self.d)

    def process_uConv(self):
        self.uConv = self.control_model.process_uConv(self.uConv)

    def update_subsys(self):
        self.update_x()
        self.update_y()
        self.update_Z()

    def update_V(self, obj):
        # TODO: figure out how to make this work when z is more than one value
        for i, upstream in enumerate(self.upstream):
            if upstream=='None':
                self.V = np.zeros((self.nyNy, 1))
            else:
                # print('########## ', obj.subsystems[upstream].control_model.Z_idn)
                # print('########## ', np.where(np.array(obj.subsystems[upstream].downstream) == self.idn)[0][0])
                print("=========================")
                print('self.idn: ', self.idn)
                # print('upstream: ', upstream)
                # self.subsystems[up].downstream.append(down)
                # idx = np.where(np.array(obj.subsystems[upstream].control_model.Z_idn) == self.idn)[0][0]
                idx = np.where(np.array(obj.subsystems[upstream].downstream) == self.idn)[0][0]
                # idx_range = len(obj.subsystems[upstream].control_model.Z_idn)
                idx_range = len(obj.subsystems[upstream].downstream)
                print('i: ', i)
                print('num_upstream: ', self.control_model.num_upstream)
                print('upstream: ', upstream)
                print('idx: ', idx)
                print('upstreams downstream: ', obj.subsystems[upstream].downstream)
                print('idx_range: ', idx_range)
                print('values: ', obj.subsystems[upstream].Z)
                print('subvalues: ', obj.subsystems[upstream].Z[idx::idx_range])
                # lkj
                self.V[i::self.control_model.num_upstream] = obj.subsystems[upstream].Z[idx::idx_range]
                print('self.V: ', self.V)
                print("=========================")
                # self.V = np.array([[val] for \
                #     val in obj.subsystems[upstream].Z])

    # def update_Psi(self, obj):
    #     # TODO: figure out how to make this work when z is more than one value
    #     for i, upstream in enumerate(self.upstream):
    #         if upstream=='None':
    #             self.Psi = np.zeros((self.nxMz, 1))
    #         else:
    #             idx = np.where(np.array(obj.subsystems[upstream].control_model.Z_idn) == self.idn)[0][0]
    #             idx_range = len(obj.subsystems[upstream].control_model.Z_idn)
    #             self.Psi[i::self.control_model.num_upstream] = obj.subsystems[upstream].gamma[idx::idx_range]
    #             # self.Psi = obj.subsystems[upstream].Gamma
    #     print('*** 3, Psi: ', self.Psi)
    
    def update_Psi(self, obj):
        # TODO: figure out how to make this work when z is more than one value
        for i, downstream in enumerate(self.downstream):
            if downstream=='None':
                self.Psi = np.zeros((self.nxMz, 1))
            else:
                idx = np.where(np.array(obj.subsystems[downstream].upstream) == self.idn)[0][0]
                idx_range = len(obj.subsystems[downstream].upstream)
                self.Psi[i::self.control_model.num_downstream] = obj.subsystems[downstream].gamma[idx::idx_range]
                # self.Psi = obj.subsystems[upstream].Gamma
        # if self.idn == 0:
        # print('subsys idn: ', self.idn)
        # print('*** 3, Psi: ', self.Psi)

    def update_x(self):
        # TODO: add in self.d to class
        # print('shape of A:', np.shape(self.A))
        # print('shape of x0:', np.shape(self.x0))
        # print('shape of Bu:', np.shape(self.Bu))
        # print('shape of self.uConv[0:self.nyBu]:', np.shape(self.uConv[0:self.nyBu]))
        # print('shape of Bv:', np.shape(self.Bv))
        # print('shape of V[0:len(self.upstream)]:', np.shape(self.V[0:len(self.upstream)]))
        # print('shape of Bd:', np.shape(self.Bd))
        # print('shape of d:', np.shape(self.d))
        # print('shape of control_model.Bd_mean_inputs:', np.shape(self.control_model.Bd_mean_inputs))
        self.x1 = dot(self.A, self.x0) \
            + dot(self.Bu, np.array(self.uConv[0:self.nyBu])) \
            + dot(self.Bv, self.V[0:len(self.upstream)]) + dot(self.Bd, self.d - self.control_model.Bd_mean_inputs)
        self.x0 = self.x1
        # print('self.x0 after update_x: ', self.x0)

    def update_y(self):
        # print('x0 statessssssssssssssssss: ', self.x0)
        # print('********************UPDATE_y: ', tp(self.uConv[0:self.nyDyu]))
        # print('UPDATE y SELF.d: ', self.d - self.control_model.Bd_mean_inputs)
        # print('shape of Cy:', np.shape(self.Cy))
        # print('shape of x0:', np.shape(self.x0))
        # print('shape of Dyu:', np.shape(self.Dyu))
        # print('shape of self.uConv[0:self.nyDyu]:', np.shape(self.uConv[0:self.nyDyu]))
        # print('shape of Dyv:', np.shape(self.Dyv))
        # print('shape of V[0:len(self.upstream)]:', np.shape(self.V[0:len(self.upstream)]))
        # print('shape of Dyd:', np.shape(self.Dyd))
        # print('shape of d:', np.shape(self.d))
        # print('shape of control_model.Bd_mean_inputs:', np.shape(self.control_model.Bd_mean_inputs))
        # print('shape of control_model.Cy_mean_outputs:', np.shape(self.control_model.Cy_mean_outputs))
        self.y = dot(self.Cy, self.x0) \
            + dot(self.Dyu, self.uConv[0:self.nyDyu]) \
            + dot(self.Dyv, self.V[0:len(self.upstream)]) \
            + dot(self.Dyd, self.d - self.control_model.Bd_mean_inputs) \
            + self.control_model.Cy_mean_outputs
        # print('self.y: ', self.y)
        # print('shape of self.y: ', np.shape(self.y))

        # self.y[1] = self.y[1] - dot(self.Dyu[1], tp(np.array([0., self.control_model.ms_dot_lin, self.control_model.T_sa_lin])))
        # self.y[1] = self.y[1] - dot(self.Dyd[1], np.array([self.control_model.T_oa_lin, 0., 0.]))
        # self.y[1] = self.y[1] - dot(self.Cy[1], self.control_model.T_z_lin)

        # print('shape of Cy_lin: ', np.shape(self.control_model.Cy_lin))
        # print('shape of Dyu_lin: ', np.shape(self.control_model.Dyu_lin))
        # print('Dyu_lin: ', self.control_model.Dyu_lin)
        # print('shape of Dyd_lin: ', np.shape(self.control_model.Dyd_lin))
        self.y = self.y - self.control_model.Cy_lin - self.control_model.Dyu_lin - self.control_model.Dyd_lin
        # print('self.y: ', self.y)
        # print('shape of self.y: ', np.shape(self.y))
        # print('sadfasdfasd: ', self.y)

    def update_Y(self):
        # print('UPDATEY SELF.D: ', self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ))
        # print('Fy: ', np.shape(self.Fy))
        # print('x0: ', np.shape(self.x0))
        # print('My: ', np.shape(self.My))
        # print('Ny: ', np.shape(self.Ny))
        # print('V: ', np.shape(self.V))
        # print('uOpt: ', np.shape(self.uOpt))
        # print('D: ', np.shape(self.D))
        # print('1: ', np.shape(dot(self.Fy, self.x0)))
        # print('2: ', np.shape(dot(self.My, self.uOpt)))
        # print('3: ', np.shape(dot(self.Ny, self.V)))
        # print('4: ', np.shape(dot(
        #         self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, (self.horiz_len, 1)
        #         )
        #     )))
        # print('5: ', np.shape(np.tile(self.control_model.Cy_mean_outputs, self.horiz_len).reshape(np.shape(self.Py)[0], 1)))

        # print(self.Fy)
        # print(self.x0)
        # self.Y = dot(self.Fy, self.x0) + dot(self.My, self.uOpt) \
        #     + dot(self.Ny, self.V) \
        #     + dot(
        #         self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ).reshape(np.shape(self.Py)[1], 1)
        #     ) \
        #     + np.tile(self.control_model.Cy_mean_outputs, self.horiz_len).reshape(np.shape(self.Py)[0], 1)
        # print('!!!!!!!!!: ', np.tile(self.control_model.Cy_mean_outputs, (self.horiz_len, 1)))
        # print('*********: ', np.tile(
                #     self.control_model.Bd_mean_inputs, (self.horiz_len, 1)
                # ))
        self.Y = dot(self.Fy, self.x0) + dot(self.My, self.uOpt) \
            + dot(self.Ny, self.V) \
            + dot(
                self.Py, self.D - np.tile(
                    self.control_model.Bd_mean_inputs, (self.horiz_len, 1)
                )
            ) \
            + np.tile(self.control_model.Cy_mean_outputs, (self.horiz_len, 1))

            
        
        # print(np.shape(self.Y))
        # print('//////: ', self.control_model.Dyd_lin)
        # print('######: ', np.tile(self.control_model.Dyd_lin, (self.horiz_len,1)))
        self.Y = self.Y \
               - np.tile(self.control_model.Cy_lin, (self.horiz_len, 1)) \
               - np.tile(self.control_model.Dyu_lin, (self.horiz_len, 1)) \
               - np.tile(self.control_model.Dyd_lin, (self.horiz_len, 1))
        # print('!!!!!!!!!: ', self.Y)
        # print('shape Cy_lin: ', self.control_model.Cy_lin)
        # print('shape Dyu_lin: ', self.control_model.Dyu_lin)
        # print('shape Dyd_lin: ', self.control_model.Dyd_lin)

        #TODO: print system matrices and compare between Y and Z

    def update_Z(self):
        # print('UPDATEZ SELF.D: ', self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ))
        # print('UPDATEZ uconv: ', self.V)
        # print('Fz: ', np.shape(self.Fz))
        # print('x0: ', np.shape(self.x0))
        # print('Mz: ', np.shape(self.Mz))
        # print('uConv: ', np.shape(self.uConv))
        # print('Pz: ', np.shape(self.Pz))
        # print('D: ', np.shape(self.D))
        # print('1: ', np.shape(dot(self.Fz, self.x0)))
        # print('2: ', np.shape(dot(self.Mz, self.uConv)))
        # print('3: ', np.shape(dot(self.Nz, self.V)))
        # print('4: ', np.shape(dot(
        #         self.Pz, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ).reshape(np.shape(self.Pz)[1], 1)
        #     )))
        # print('5: ', np.shape(np.tile(self.control_model.Bd_mean_inputs, self.horiz_len).reshape(np.shape(self.Pz)[1], 1)))
        # self.Z = dot(self.Fz, self.x0) + dot(self.Mz, self.uConv) \
        #     + dot(self.Nz, self.V) + dot(
        #         self.Pz, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ).reshape(np.shape(self.Pz)[1], 1)
        #     ) \
        #     + np.tile(self.control_model.Cz_mean_outputs, self.horiz_len).reshape(np.shape(self.Pz)[0], 1)

        self.Z = dot(self.Fz, self.x0) + dot(self.Mz, self.uConv) \
            + dot(self.Nz, self.V) + dot(
                self.Pz, self.D - np.tile(
                    self.control_model.Bd_mean_inputs, (self.horiz_len, 1)
                )
            ) \
            + np.tile(self.control_model.Cz_mean_outputs, self.horiz_len).reshape(np.shape(self.Pz)[0], 1)
        # print('shape Cz_lin: ', self.control_model.Cz_lin)
        # print('shape Dzu_lin: ', self.control_model.Dzu_lin)
        # print('shape Dzd_lin: ', self.control_model.Dzd_lin)
        # print('shape of Z: ', np.shape(self.Z))
        # print('Z before: ', self.Z)
        self.Y = self.Y \
               - np.tile(self.control_model.Cy_lin, (self.horiz_len, 1)) \
               - np.tile(self.control_model.Dyu_lin, (self.horiz_len, 1)) \
               - np.tile(self.control_model.Dyd_lin, (self.horiz_len, 1))

        # if self.idn == 0:
        #     print('###################### before self.Z: ', self.Z)
        self.Z = self.Z \
                - np.tile(self.control_model.Cz_lin, self.horiz_len).reshape(np.shape(self.Z)[0], np.shape(self.Z)[1]) \
                - np.tile(self.control_model.Dzu_lin, self.horiz_len).reshape(np.shape(self.Z)[0], np.shape(self.Z)[1]) \
                - np.tile(self.control_model.Dzd_lin, self.horiz_len).reshape(np.shape(self.Z)[0], np.shape(self.Z)[1])
        # if self.idn == 0:
        #     print('###################### after self.Z: ', self.Z)
        # print('Z after: ', self.Z)
        # print('size of self.Z: ', np.shape(self.Z))

    def sys_matrices(self):       
        # self.Fy = np.array([dot(self.Cy, matpower(self.A, i)) \
        #           for i in range(1, self.horiz_len + 1)])
        # print('idn: ', self.idn)
        self.Fy = np.array([dot(self.Cy, matpower(self.A, i)) \
                  for i in range(0, self.horiz_len)])
        self.Fy = np.reshape(
            self.Fy, (self.nxCy*self.horiz_len, self.nxA), order='C'
        )
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
        #             dot(
        #                 matpower(self.A, i), self.Bu )), Mytmp[:-self.nyBu])))
        #             )
        #     else:
        #         if dot(self.Cy, dot(matpower(self.A, i), self.Bu )).ndim == 1:
        #             Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
        #                 dot(matpower(self.A, i), self.Bu )), 
        #                       Mytmp[-self.nxCy:,:-self.nyBu][0]))))
        #         else:
        #             Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
        #                 dot(matpower(self.A, i), self.Bu )), 
        #                       Mytmp[-self.nxCy:,:-self.nyBu]))))
        # self.My = Mytmp

        Mytmp0 = self.Dyu
        Mytmp0Shape = np.shape(Mytmp0)
        # print('shape Dyu: ', np.shape(self.Dyu))
        # print('shape Cy: ', np.shape(self.Cy))
        # print('shape Bu: ', np.shape(self.Bu))
        # print('shape Mytmp0: ', np.shape(Mytmp0))
        # print('self.idn: ', self.idn)
        # print('Cy: ', self.Cy)
        # print('Bu: ', self.Bu)
        # print('Mytmp0: ', Mytmp0)
        # print('idn:', self.idn)
        Mytmp = np.hstack((dot(self.Cy, self.Bu), Mytmp0))
        # print('shape Mytmp: ', np.shape(Mytmp))
        for i in range(self.horiz_len - 1):
            Mytmp0 = np.hstack((Mytmp0, np.zeros(Mytmp0Shape)))
        for i in range(self.horiz_len - 2):
            Mytmp = np.hstack((Mytmp, np.zeros(Mytmp0Shape)))
        Mytmp = np.vstack((Mytmp0, Mytmp))
        # Mytmp = dot(self.Cy, self.Bu)
        # # print('Mytmp: ', Mytmp)
        # MytmpShape = np.shape(Mytmp)
        # Mytmp = np.hstack((Mytmp, self.Dyu))
        # # print('(((((((((((((((( combo of CyBU and Dy: ', Mytmp)
        # for i in range(self.horiz_len - 2):
        #     Mytmp = np.hstack((Mytmp, np.zeros(MytmpShape)))
        # print('////////////////// 1st row of My: ', Mytmp)
        for i in range(1, self.horiz_len - 1):
            if Mytmp.ndim == 1:
                Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
                    dot(matpower(self.A, i), self.Bu )), Mytmp[:-self.nyBu+1]))))
            else:
                if dot(self.Cy, dot(matpower(self.A, i), self.Bu )).ndim == 1:
                    Mytmp = np.vstack(
                        (Mytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bu )),
                            Mytmp[-self.nxCy:,:-self.nyBu][0])
                        ))
                    )
                else:
                    Mytmp = np.vstack(
                        (Mytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bu )),
                            Mytmp[-self.nxCy:,:-self.nyBu])
                        ))
                    )
        self.My = Mytmp
        # print('************* My: ', self.My)

        Mztmp0 = self.Dzu
        Mztmp0Shape = np.shape(Mztmp0)
        Mztmp = np.hstack((dot(self.Cz, self.Bu), Mztmp0))
        for i in range(self.horiz_len - 1):
            Mztmp0 = np.hstack((Mztmp0, np.zeros(Mztmp0Shape)))
        # print('Mztmp0: ', Mztmp0)
        for i in range(self.horiz_len - 2):
            Mztmp = np.hstack((Mztmp, np.zeros(Mztmp0Shape)))
        # print('Mztmp: ', Mztmp)
        Mztmp = np.vstack((Mztmp0, Mztmp))
        # print('hhhhhhhhhhhhhhhh: ', Mztmp)

        for i in range(1, self.horiz_len - 1):
            if Mztmp.ndim == 1:
                Mztmp = np.vstack(
                    (Mztmp,
                    np.hstack(
                        (dot(self.Cz, dot(matpower(self.A, i), self.Bu )),
                        Mztmp[-self.nyBu+1:])
                    ))
                )
            else:
                if dot(self.Cz, dot(matpower(self.A, i), self.Bu )).ndim == 1:
                    Mztmp = np.vstack(
                        (Mztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bu )),
                            Mztmp[-self.nxCz:,:-self.nyBu][0])
                        ))
                    )
                else:
                    Mztmp = np.vstack(
                        (Mztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bu )),
                            Mztmp[-self.nxCz:,:-self.nyBu])
                        ))
                    )
        self.Mz = Mztmp
        # print('/////////// Mz: ', self.Mz)

        Nytmp0 = self.Dyv
        Nytmp0Shape = np.shape(Nytmp0)
        Nytmp = np.hstack((dot(self.Cy, self.Bv), Nytmp0))
        # print('Nytmp: ', Nytmp)
        # print('Nytmp0: ', Nytmp0)
        for i in range(self.horiz_len - 1):
            Nytmp0 = np.hstack((Nytmp0, np.zeros(Nytmp0Shape)))
        for i in range(self.horiz_len - 2):
            Nytmp = np.hstack((Nytmp, np.zeros(Nytmp0Shape)))
        # print('Nytmp: ', Nytmp)
        # print('Nytmp0: ', Nytmp0)
        Nytmp = np.vstack((Nytmp0, Nytmp))
        
        for i in range(1, self.horiz_len - 1):
            if Nytmp.ndim == 1:
                Nytmp = np.vstack((Nytmp, np.hstack((dot(self.Cy, \
                    dot(matpower(self.A, i), self.Bv )), Nytmp[:-self.nyBv]))))
            else:
                if dot(self.Cy, dot(matpower(self.A, i), self.Bv )).ndim == 1:
                    Nytmp = np.vstack(
                        (Nytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bv )),
                            Nytmp[-self.nxCy:,:-self.nyBv][0])
                    ))
                )
                else:
                    Nytmp = np.vstack(
                        (Nytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bv )),
                            Nytmp[-self.nxCy:,:-self.nyBv])
                        ))
                    )
        self.Ny = Nytmp

        Nztmp0 = self.Dzv
        Nztmp0Shape = np.shape(Nztmp0)
        Nztmp = np.hstack((dot(self.Cz, self.Bv), Nztmp0))
        for i in range(self.horiz_len - 1):
            Nztmp0 = np.hstack((Nztmp0, np.zeros(Nztmp0Shape)))
        for i in range(self.horiz_len - 2):
            Nztmp = np.hstack((Nztmp, np.zeros(Nztmp0Shape)))
        Nztmp = np.vstack((Nztmp0, Nztmp))
        
        # Nztmp = dot(self.Cz, self.Bv)
        # NztmpShape = np.shape(Nztmp)
        # for i in range(self.horiz_len - 1):
        #     Nztmp = np.hstack((Nztmp, np.zeros(NztmpShape)))
        # Nztmp0 = np.zeros(np.shape(Nztmp))
        # Nztmp = np.vstack((Nztmp0, Nztmp))
        for i in range(1, self.horiz_len - 1):
            if Nztmp.ndim == 1:
                Nztmp = np.vstack((Nztmp, np.hstack((dot(self.Cz, \
                    dot(matpower(self.A, i), self.Bv )), Nztmp[:-self.nyBv]))))
            else:
                if dot(self.Cz, dot(matpower(self.A, i), self.Bv )).ndim == 1:
                    Nztmp = np.vstack(
                        (Nztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bv )),
                            Nztmp[-self.nxCz:,:-self.nyBv][0])
                        ))
                    )
                else:
                    Nztmp = np.vstack(
                        (Nztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bv )),
                            Nztmp[-self.nxCz:,:-self.nyBv])
                        ))
                    )
        self.Nz = Nztmp


        Pytmp0 = self.Dyd
        Pytmp0Shape = np.shape(Pytmp0)
        Pytmp = np.hstack((dot(self.Cy, self.Bd), Pytmp0))
        for i in range(self.horiz_len - 1):
            Pytmp0 = np.hstack((Pytmp0, np.zeros(Pytmp0Shape)))
        for i in range(self.horiz_len - 2):
            Pytmp = np.hstack((Pytmp, np.zeros(Pytmp0Shape)))
        Pytmp = np.vstack((Pytmp0, Pytmp))

        # Pytmp = dot(self.Cy, self.Bd)
        # PytmpShape = np.shape(Pytmp)
        # for i in range(self.horiz_len - 1):
        #     Pytmp = np.hstack((Pytmp, np.zeros(PytmpShape)))

        for i in range(1, self.horiz_len - 1):
            if Pytmp.ndim == 1:
                Pytmp = np.vstack((Pytmp, np.hstack((dot(self.Cy, \
                    dot(matpower(self.A, i), self.Bd )), Pytmp[:-self.nyBd]))))
            else:
                if dot(self.Cy, dot(matpower(self.A, i), self.Bd )).ndim == 1:
                    Pytmp = np.vstack(
                        (Pytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bd )),
                            Pytmp[-self.nxCy:,:-self.nyBd][0])
                    ))
                )
                else:
                    Pytmp = np.vstack(
                        (Pytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bd )),
                            Pytmp[-self.nxCy:,:-self.nyBd])
                        ))
                    )
        self.Py = Pytmp


        # print('shape Dzd: ', np.shape(self.Dzd))
        # print('shape Cz: ', np.shape(self.Cz))
        # print('shape Bd: ', np.shape(self.Bd))
        Pztmp0 = self.Dzd
        Pztmp0Shape = np.shape(Pztmp0)
        Pztmp = np.hstack((dot(self.Cz, self.Bd), Pztmp0))
        for i in range(self.horiz_len - 1):
            Pztmp0 = np.hstack((Pztmp0, np.zeros(Pztmp0Shape)))
        for i in range(self.horiz_len - 2):
            Pztmp = np.hstack((Pztmp, np.zeros(Pztmp0Shape)))
        # print('shape Pztmp0: ', np.shape(Pztmp0))
        # print('shape Pztmp: ', np.shape(Pztmp))
        Pztmp = np.vstack((Pztmp0, Pztmp))

        # Pztmp = dot(self.Cz, self.Bd)
        # PztmpShape = np.shape(Pztmp)
        # for i in range(self.horiz_len - 1):
        #     Pztmp = np.hstack((Pztmp, np.zeros(PztmpShape)))
        # Pztmp0 = np.zeros(np.shape(Pztmp))
        # Pztmp = np.vstack((Pztmp0, Pztmp))

        for i in range(1, self.horiz_len - 1):
            if Pztmp.ndim == 1:
                Pztmp = np.vstack((Pztmp, np.hstack((dot(self.Cz, \
                    dot(matpower(self.A, i), self.Bd )), Pztmp[:-self.nyBd]))))
            else:
                if dot(self.Cz, dot(matpower(self.A, i), self.Bd )).ndim == 1:
                    Pztmp = np.vstack(
                        (Pztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bd )),
                            Pztmp[-self.nxCz:,:-self.nyBd][0])
                        ))
                    )
                else:
                    Pztmp = np.vstack(
                        (Pztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bd )),
                            Pztmp[-self.nxCz:,:-self.nyBd])
                        ))
                    )
        self.Pz = Pztmp

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

        if self.Py.ndim == 1:
            self.nxPy = 1
            self.nyPy = 0
        else:
            self.nxPy = np.shape(self.Py)[0]
            self.nyPy = np.shape(self.Py)[1]

        if self.Pz.ndim == 1:
            self.nxPz = 1
            self.nyPz = 0
        else:
            self.nxPz = np.shape(self.Pz)[0]
            self.nyPz = np.shape(self.Pz)[1]

        # TODO: make this work for user-supplied Q's and S's
        self.Q = np.diag(np.ones(self.nxMy))
        self.Q = self.control_model.process_Q(self.Q)
        
        self.S = np.diag(np.ones(self.nyMy))
        self.S = self.control_model.process_S(self.S)

    def mat_sizes(self):
        if self.A.ndim == 1:
            self.nxA = 1
            self.nyA = 1
        elif self.A.ndim == 2:
            self.nxA = np.shape(self.A)[0]
            self.nyA = np.shape(self.A)[1]
        else:
            raise Exception("The 'A' matrix has > 2 dimensions")

        if self.Bu.ndim == 1:
            self.nxBu = 1
            self.nyBu = 1
        elif self.Bu.ndim == 2:
            self.nxBu = np.shape(self.Bu)[0]
            self.nyBu = np.shape(self.Bu)[1]
        else:
            raise Exception("The 'Bu' matrix has > 2 dimensions")

        if self.Bv.ndim == 1:
            self.nxBv = 1
            self.nyBv = 1
        elif self.Bv.ndim == 2:
            self.nxBv = np.shape(self.Bv)[0]
            self.nyBv = np.shape(self.Bv)[1]
        else:
            raise Exception("The 'Bv' matrix has > 2 dimensions")

        if self.Bd.ndim == 1:
            self.nxBd = 1
            self.nyBd = 1
        elif self.Bd.ndim == 2:
            self.nxBd = np.shape(self.Bd)[0]
            self.nyBd = np.shape(self.Bd)[1]
        else:
            raise Exception("The 'Bd' matrix has > 2 dimensions")

        if self.Cy.ndim == 1:
            self.nxCy = 1
            self.nyCy = 1
        elif self.Cy.ndim == 2:
            self.nxCy = np.shape(self.Cy)[0]
            self.nyCy = np.shape(self.Cy)[1]
        else:
            raise Exception("The 'Cy' matrix has > 2 dimensions")

        if self.Cz.ndim == 1:
            self.nxCz = 1
            self.nyCz = 1
        elif self.Cz.ndim == 2:
            self.nxCz = np.shape(self.Cz)[0]
            self.nyCz = np.shape(self.Cz)[1]
        else:
            raise Exception("The 'Cz' matrix has > 2 dimensions")

        if self.Dyu.ndim == 1:
            self.nxDyu = 1
            self.nyDyu = 1
        elif self.Dyu.ndim == 2:
            self.nxDyu = np.shape(self.Dyu)[0]
            self.nyDyu = np.shape(self.Dyu)[1]
        else:
            raise Exception("The 'Dy' matrix has > 2 dimensions")

        if self.Dyv.ndim == 1:
            self.nxDyv = 1
            self.nyDyv = 1
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

